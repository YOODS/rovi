#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <pylonc/PylonC.h>

#include	"cparser.h"

#define CHECK( errc ) if ( GENAPI_E_OK != errc ) printErrorAndExit( errc )

void printErrorAndExit( GENAPIC_RESULT errc );

int sockd;

static PYLON_DEVICE_HANDLE	hDev;
static int32_t payloadSize;
static unsigned char *imgBuf;

int grabOnce(PylonGrabResult_t *gRes,int offset){
	GENAPIC_RESULT res;
	_Bool bufferReady;
	res = PylonDeviceGrabSingleFrame( hDev, 0, imgBuf+offset, payloadSize, gRes, &bufferReady, 6000000L);
	CHECK(res);
	if( GENAPI_E_OK == res && !bufferReady ){/* 500ms Timeout */
		return -2; 
	}
	else if( gRes->Status == Grabbed ){
		int sz=(int)gRes->SizeX*gRes->SizeY;
		return sz;
	}
	else if ( gRes->Status == Failed ){
		return -1;
	}
}

void *grab(void *x){
	int i=0;
	PylonGrabResult_t grabResult;
	for (;;i++){
		if(i&1){
			int sz=grabOnce(&grabResult,0);
			if(sz>0){
				fprintf(stdout,"{\"capt\":%d}\n",0);
				fflush(stdout);
			}
		}
		else{
			int sz=grabOnce(&grabResult,payloadSize);
			if(sz>0){
				fprintf(stdout,"{\"capt\":%d}\n",payloadSize);
				fflush(stdout);
			}
		}
	}
}

int main(int argc,char **argv){
	PylonGrabResult_t grabResult;
	GENAPIC_RESULT			res;           /* Return value of pylon methods. */
	size_t					numDevices;    /* Number of available devices. */
	_Bool						isAvail;
	int						i;
	pthread_t				thread;
	int						thret;

	PylonInitialize();

	res = PylonEnumerateDevices( &numDevices );
	CHECK(res);
	if (!numDevices){
		fprintf( stderr, "No devices found!\n" );
		PylonTerminate();
		exit(EXIT_FAILURE);
	}

	res = PylonCreateDeviceByIndex( 0, &hDev );
	CHECK(res);
	res = PylonDeviceOpen( hDev, PYLONC_ACCESS_MODE_CONTROL | PYLONC_ACCESS_MODE_STREAM );
	CHECK(res);

	if(PylonDeviceFeatureIsAvailable(hDev, "EnumEntry_PixelFormat_Mono8")){
		res = PylonDeviceFeatureFromString(hDev, "PixelFormat", "Mono8" );
		CHECK(res);
	}
	if(PylonDeviceFeatureIsWritable(hDev, "GevSCPSPacketSize")){
		res = PylonDeviceSetIntegerFeature( hDev, "GevSCPSPacketSize", 1500 );
		CHECK(res);
	}
	if(PylonDeviceFeatureIsReadable(hDev, "PayloadSize")){
		res = PylonDeviceGetIntegerFeatureInt32( hDev, "PayloadSize", &payloadSize );
		CHECK(res);
	}
	else{
		PYLON_STREAMGRABBER_HANDLE  hGrabber;
		NODEMAP_HANDLE hStreamNodeMap;
		NODE_HANDLE hNode;
		int64_t i64payloadSize;

		res = PylonDeviceGetStreamGrabber( hDev, 0, &hGrabber );
		CHECK(res);
		res = PylonStreamGrabberOpen( hGrabber );
		CHECK(res);

		res = PylonStreamGrabberGetNodeMap(hGrabber, &hStreamNodeMap);
		CHECK(res);
		res = GenApiNodeMapGetNode( hStreamNodeMap, "PayloadSize", &hNode );
		CHECK(res);
		if ( GENAPIC_INVALID_HANDLE == hNode ){
			fprintf( stderr, "There is no PayloadSize parameter.\n");
			PylonTerminate();
			return EXIT_FAILURE;
		}
		res = GenApiIntegerGetValue(hNode, &i64payloadSize);
		CHECK(res);
		payloadSize = (int32_t) i64payloadSize;

		res = PylonStreamGrabberClose( hGrabber );
		CHECK(res);
	}
//	if((sockd=shm_open("/rovi_camera",O_CREAT|O_RDWR,0666))<0){
	if((sockd=shmget(IPC_PRIVATE,2*payloadSize,IPC_CREAT|0666))<0){
		perror("shm open");
		exit(-1);
	}
//	ftruncate(sockd,payloadSize);
//	if((int64_t)(imgBuf=(unsigned char *)mmap(0,payloadSize,PROT_READ|PROT_WRITE,MAP_SHARED,sockd,0))<0){
	if((int64_t)(imgBuf=(unsigned char *)shmat(sockd,NULL,0))<0){
		perror("mmap");
		exit(-1);
	}
	imgBuf[0]=1;//try writing onto shared memory
	fprintf(stdout,"{\"shm\":%d}\n",sockd);
	fflush(stdout);

	PylonDeviceFeatureFromString( hDev, "TriggerMode", "On");
	PylonDeviceSetBooleanFeature( hDev, "AcquisitionFrameRateEnable", 1 );

	thret = pthread_create( &thread, NULL, grab, NULL);
	fprintf(stderr,"Grab thread: %d\n",thret);

	for(;;){
		argv=cparser(stdin);
		if(argv==NULL) continue;
		if(!(int)strcmp(argv[0],"quit")) break;
		res=PylonDeviceFeatureFromString( hDev, argv[0], argv[1] );
	}

	res = PylonDeviceClose(hDev);
	CHECK(res);
	res = PylonDestroyDevice(hDev);
	CHECK(res);
	fprintf(stderr,"Disconnect Camera\n");
 
	PylonTerminate();

	return EXIT_SUCCESS;
}

void printErrorAndExit( GENAPIC_RESULT errc ){
	PylonTerminate();  /* Releases all pylon resources. */
	exit(EXIT_FAILURE);
}
