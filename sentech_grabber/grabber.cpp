#include <stdio.h>
#include <sys/shm.h>
#include <atomic>
#include <iomanip>    //for setprecision
#include <thread>
#include <chrono>
#include <StApi_TL.h>

#include	"cparser.h"

#define DEBUG 0

#if DEBUG
#define DEBUG_TO_CERR(x) cerr << x << flush;
#else
#define DEBUG_TO_CERR(x)
#endif

using namespace GenApi;
using namespace StApi;
using namespace std;

const uint64_t nCountOfImagesToGrab = GENTL_INFINITE;

int sockd;

static int32_t payloadSize;
static unsigned char *imgBuf;

static atomic<bool> exit_flag(false);

void acquisitionWorker(const gcstring cam_l_name, const gcstring cam_r_name, CIStDevicePtrArray *ppIStDeviceList, CIStDataStreamPtrArray *ppIStDataStreamList)
{
#if DEBUG
  auto beforealltm = chrono::system_clock::now();
  auto beforeLtm = beforealltm;
  auto beforeRtm = beforealltm;
#endif

  try
  {
    DEBUG_TO_CERR("StartAcquisition" << endl)

    ppIStDataStreamList->StartAcquisition(nCountOfImagesToGrab);
    ppIStDeviceList->AcquisitionStart();

    CIStStreamBufferPtr pIStStreamBuffer;

    while (ppIStDataStreamList->IsGrabbingAny())
    {
      // DEBUG_TO_CERR("before RetrieveBuffer" << endl)

      try
      {
        pIStStreamBuffer = ppIStDataStreamList->RetrieveBuffer(1000); // TODO 500  1000  0xFFFFFFFF
      }
      catch (const GenICam::TimeoutException &e)
      {
        if (exit_flag)
        {
          DEBUG_TO_CERR("after RetrieveBuffer timeout, exit_flag is true. acquisitionWorker break." << endl)
          break;
        }
        else {
          pIStStreamBuffer = NULL;
        }
      }

      // DEBUG_TO_CERR("after  RetrieveBuffer" << endl)

      if (pIStStreamBuffer)
      {
        if ((pIStStreamBuffer) && (pIStStreamBuffer->GetIStStreamBufferInfo()->IsImagePresent()))
        {
          IStImage *pIStImage = pIStStreamBuffer->GetIStImage();

          const gcstring cur_cam_name = pIStStreamBuffer->GetIStDataStream()->GetIStDevice()->GetIStDeviceInfo()->GetDisplayName();

          DEBUG_TO_CERR(
            cur_cam_name
            << " : BlockId=" << dec << pIStStreamBuffer->GetIStStreamBufferInfo()->GetFrameID()
            << " Size:" << pIStImage->GetImageWidth() << " x " << pIStImage->GetImageHeight()
            << " First byte =" << (uint32_t)*(uint8_t*)pIStImage->GetImageBuffer()
            << " " << setprecision(4) << pIStStreamBuffer->GetIStDataStream()->GetCurrentFPS() << "FPS" << endl
          )

          // TODO  AllocateStreamBuffersManuallyサンプルでのAllocate(pCreatedBuffer)に共有メモリアドレスを返せば、memcpyの時間をはぶけそう
          if (cur_cam_name == cam_l_name)
          {
            // DEBUG_TO_CERR("before cam_l memcpy" << endl)
            memcpy(imgBuf, pIStImage->GetImageBuffer(), payloadSize);
            cout << "{\"capt\":" << 0 << "}" << endl;
#if DEBUG
            auto curtm = chrono::system_clock::now();
            auto elapsedall_msec = chrono::duration_cast<chrono::milliseconds>(curtm - beforealltm);
            auto elapsedL_msec = chrono::duration_cast<chrono::milliseconds>(curtm - beforeLtm);
            DEBUG_TO_CERR("flush " << "{\"capt\":" << 0 << "} done. elapsed_msec all=" << elapsedall_msec.count() << " L=" << elapsedL_msec.count() << endl)
            beforealltm = curtm;
            beforeLtm = curtm;
#endif
          }
          else if (cur_cam_name == cam_r_name)
          {
            // DEBUG_TO_CERR("before cam_r memcpy" << endl)
            memcpy(imgBuf + payloadSize, pIStImage->GetImageBuffer(), payloadSize);
            cout << "{\"capt\":" << payloadSize << "}" << endl;
#if DEBUG
            auto curtm = chrono::system_clock::now();
            auto elapsedall_msec = chrono::duration_cast<chrono::milliseconds>(curtm - beforealltm);
            auto elapsedR_msec = chrono::duration_cast<chrono::milliseconds>(curtm - beforeRtm);
            DEBUG_TO_CERR("flush " << "{\"capt\":" << payloadSize << "} done. elapsed_msec all=" << elapsedall_msec.count() << " R=" << elapsedR_msec.count() << endl)
            beforealltm = curtm;
            beforeRtm = curtm;
#endif
          }
          else
          {
            cerr << "ERROR: unknown cam_name[" << cur_cam_name <<"]" << endl;
          }
        }
        else
        {
          cerr << "ERROR: Image data does not exist" << endl;
        }
      }

      if (exit_flag)
      {
        DEBUG_TO_CERR("exit_flag is true. acquisitionWorker break." << endl)
        break;
      }
    }

    ppIStDeviceList->AcquisitionStop();
    ppIStDataStreamList->StopAcquisition();

    DEBUG_TO_CERR("StopAcquisition" << endl)
  }
  catch (const GenICam::GenericException &e)
  {
    cerr << endl << "ERROR: In acquisitionWorker, an exception occurred." << endl << e.GetDescription() << endl;
  }

  DEBUG_TO_CERR("acquisitionWorker end." << endl)
}

int main(int argc, char **argv)
{
  DEBUG_TO_CERR("grabber start" << endl << endl)

  gcstring camLname;
  gcstring camRname;

  if (argc != 3)
  {
    cerr << "Usage:   STGENTL_GIGE_HEARTBEAT=<HeartbeatTimeout(ms)> " << argv[0] << " <Left Camera DisplayName> <Right Camera DisplayName>" << endl << endl;
    cerr << "Example: STGENTL_GIGE_HEARTBEAT=5000 " << argv[0] << " 'STC_BBE132GE(17AB755)' 'STC_BBE132GE(17AB756)'" << endl << endl;
    return EXIT_FAILURE;
  }
  else
  {
    camLname = argv[1];
    camRname = argv[2];
    DEBUG_TO_CERR("now camLname=" << camLname << " camRname=" << camRname << endl << endl)
  }

  try
  {
    CStApiAutoInit objStApiAutoInit;

    CIStSystemPtr pIStSystem(CreateIStSystem(StSystemVendor_Sentech));

    CIStDevicePtrArray pIStDeviceList;

    CIStDataStreamPtrArray pIStDataStreamList;

    do
    {
      IStDeviceReleasable *pIStDeviceReleasable = NULL;

      try
      {
        pIStDeviceReleasable = pIStSystem->CreateFirstIStDevice();
        DEBUG_TO_CERR("Device found..." << pIStDeviceReleasable->GetIStDeviceInfo()->GetDisplayName() << endl)
      }
      catch (...)
      {
        if (pIStDeviceList.GetSize() == 0)
        {
          throw;
        }
        else
        {
          DEBUG_TO_CERR("break. device list size=" << pIStDeviceList.GetSize() << endl << endl)
          break;
        }
      }

      // TODO とりあえず。ここで必須なのは set GevSCPSPacketSize, set GevSCPD, get PayloadSize
      {
        CNodeMapPtr pINodeMapRemote(pIStDeviceReleasable->GetRemoteIStPort()->GetINodeMap());

        //// set TriggerMode
        CEnumerationPtr pIEnumeration_TriggerMode(pINodeMapRemote->GetNode("TriggerMode"));

        // CEnumEntryPtr pIEnumEntry_On(pIEnumeration_TriggerMode->GetEntryByName("On"));
        // cerr << "on=" << pIEnumEntry_On->GetValue() << endl; // 1
        //
        // CEnumEntryPtr pIEnumEntry_Off(pIEnumeration_TriggerMode->GetEntryByName("Off"));
        // cerr << "off=" << pIEnumEntry_Off->GetValue() << endl; // 0

        DEBUG_TO_CERR("TriggerMode=" << pIEnumeration_TriggerMode->GetCurrentEntry()->GetSymbolic() << endl)

        // to On
        // pIEnumeration_TriggerMode->SetIntValue(pIEnumEntry_On->GetValue());
        pIEnumeration_TriggerMode->SetIntValue(pIEnumeration_TriggerMode->GetEntryByName("On")->GetValue());
        DEBUG_TO_CERR("now TriggerMode=" << pIEnumeration_TriggerMode->GetCurrentEntry()->GetSymbolic() << endl)

        //// set FPS
        CNodePtr pINodeFps(pINodeMapRemote->GetNode("AcquisitionFrameRate"));
        CFloatPtr pIFloatFps(pINodeFps);
        double dblFps = pIFloatFps->GetValue();
        DEBUG_TO_CERR("Fps=" << dblFps << endl)
        pIFloatFps->SetValue(10.0); // 動作確認環境では30でも大丈夫。ちなみに60にすると実際は40程度で動く。
        DEBUG_TO_CERR("now Fps=" << pIFloatFps->GetValue() << endl)

        //// set GevSCPSPacketSize
        CNodePtr pINodePacketSize(pINodeMapRemote->GetNode("GevSCPSPacketSize"));
        CIntegerPtr pIIntegerPacketSize(pINodePacketSize);
        int64_t intPacketSize = pIIntegerPacketSize->GetValue();
        DEBUG_TO_CERR("PacketSize=" << intPacketSize << endl)
        // GevSCPSPacketSizeはAcquisitionFrameRate設定で勝手に8420に変わるが、即座に変わるわけでもないみたい
        intPacketSize = 9000;
        pIIntegerPacketSize->SetValue(intPacketSize);
        DEBUG_TO_CERR("now PacketSize=" << pIIntegerPacketSize->GetValue() << endl)

        //// get GevTimestampTickFrequency
        CNodePtr pINodeTTF(pINodeMapRemote->GetNode("GevTimestampTickFrequency"));
        CIntegerPtr pIIntegerTTF(pINodeTTF);
        int64_t intTTF = pIIntegerTTF->GetValue();
        DEBUG_TO_CERR("TimestampTickFrequency=" << intTTF << endl)

        // TODO 本来はカメラ台数が確定してから割当帯域や転送時間を経て求める必要があるが、カメラ2台ならここでこの1行でも結果は同じになる
        double delay_time = intPacketSize / (100.0 * 1000 * 1000);
        DEBUG_TO_CERR("delay_time=" << delay_time << endl)

        //// set GevSCPD
        CNodePtr pINodeSCPD(pINodeMapRemote->GetNode("GevSCPD"));
        CIntegerPtr pIIntegerSCPD(pINodeSCPD);
        int64_t intSCPD = pIIntegerSCPD->GetValue();
        DEBUG_TO_CERR("SCPD=" << intSCPD << endl)
        pIIntegerSCPD->SetValue((int64_t)(delay_time * intTTF));
        DEBUG_TO_CERR("now GevSCPD=" << pIIntegerSCPD->GetValue() << endl)

        //// get PayloadSize
        CNodePtr pINodePayloadSize(pINodeMapRemote->GetNode("PayloadSize"));
        CIntegerPtr pIIntegerPayloadSize(pINodePayloadSize);
        int64_t intPayloadSize = pIIntegerPayloadSize->GetValue();
        DEBUG_TO_CERR("PayloadSize=" << intPayloadSize << endl)
        payloadSize = intPayloadSize;

        //// get GevHeartbeatTimeout
        CNodePtr pINodeHbTo(pINodeMapRemote->GetNode("GevHeartbeatTimeout"));
        CIntegerPtr pIIntegerHbTo(pINodeHbTo);
        int64_t intHbTo = pIIntegerHbTo->GetValue();
        DEBUG_TO_CERR("GevHeartbeatTimeout=" << intHbTo << endl)

        DEBUG_TO_CERR(endl)
      }

      pIStDeviceList.Register(pIStDeviceReleasable);

      // DEBUG_TO_CERR("GetDataStreamCount=" << pIStDeviceReleasable->GetDataStreamCount() << endl)
      DEBUG_TO_CERR("Device[" << (pIStDeviceList.GetSize() - 1) << "] registered..." << pIStDeviceReleasable->GetIStDeviceInfo()->GetDisplayName() << endl << endl)

      pIStDataStreamList.Register(pIStDeviceReleasable->CreateIStDataStream(0));

    } while (true);

    if (pIStDeviceList.GetSize() != 2)
    {
      cerr << "ERROR: found " << pIStDeviceList.GetSize() << " devices ... not 2!" << endl;
      exit(EXIT_FAILURE);
    }

    sockd = shmget(IPC_PRIVATE, 2 * payloadSize, IPC_CREAT | 0666);
    if (sockd < 0)
		{
      perror("shm open");
      exit(-1);
    }
    imgBuf = (unsigned char *)shmat(sockd, NULL, 0);
    if ((int64_t)imgBuf < 0)
    {
      perror("mmap");
      exit(-1);
    }
    imgBuf[0] = 1; // try writing onto shared memory
    cout << "{\"shm\":" << sockd << "}" << endl << endl;

    thread worker(acquisitionWorker, camLname, camRname, &pIStDeviceList, &pIStDataStreamList);


    DEBUG_TO_CERR(endl << "Please input from stdin. Input 'quit' to exit." << endl << endl)

    for(;;){
      argv=cparser(stdin);
      if(argv==NULL) continue;
      // DEBUG_TO_CERR("argv[0]=" << argv[0] << ", argv[1]=" << argv[1] << endl << endl)
      if (!(int)strcmp(argv[0],"quit"))
      {
        DEBUG_TO_CERR("now exit_flag=true" << endl)
        exit_flag = true;
        break;
      }

      if (argv[1])
      {
        for (size_t i = 0; i < pIStDeviceList.GetSize(); i++)
        {
          DEBUG_TO_CERR("Device[" << i << "](" << pIStDeviceList[i]->GetIStDeviceInfo()->GetDisplayName() << ") param update... " << argv[0] << " to " << argv[1] << endl)

          CNodeMapPtr pINodeMapRemote(pIStDeviceList[i]->GetRemoteIStPort()->GetINodeMap());

          CValuePtr pParam(pINodeMapRemote->GetNode(argv[0]));

          if (pParam.IsValid())
          {
            DEBUG_TO_CERR("Current " << argv[0] << " value=" << pParam->ToString() << endl)
            try
            {
              pParam->FromString(argv[1]);
              DEBUG_TO_CERR("Set param done. " << argv[1] << endl)
            }
            catch (const GenICam::GenericException &e)
            {
              cerr << endl << "ERROR: Set param... An exception occurred." << endl << e.GetDescription() << endl;
            }
          }
          else
          {
            cerr << "ERROR: Unable to get the param " << argv[0] << endl;
          }

          // DEBUG_TO_CERR(endl);
        }
      }

      DEBUG_TO_CERR(endl);
    }

    if (worker.joinable())
    {
      DEBUG_TO_CERR("before join" << endl)
      worker.join();
      DEBUG_TO_CERR("after  join" << endl)
    }

  }
  catch (const GenICam::GenericException &e)
  {
    cerr << endl << "ERROR: An exception occurred." << endl << e.GetDescription() << endl;
  }

  DEBUG_TO_CERR("Disconnect Camera" << endl)

  DEBUG_TO_CERR("grabber end" << endl << endl)

  return EXIT_SUCCESS;
}
