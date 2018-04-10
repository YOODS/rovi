#pragma once


#include <fstream>
#include <string>
#include <map>

class ParaConfigure {
public:
    /**
     * コンストラクタ
     */
    ParaConfigure() : para() {}


    /**
     * 前処理器の設定を行います.
     * @return 状態値.
     *   - 0: 処理成功
     *   - 1: 設定ファイルの読み込み失敗
     * @param filename 設定ファイル名
     */
    bool config(std::string filename) {
        std::ifstream ifs(filename.c_str());
        if (!ifs.is_open()) return false;

        // 設定読み込み
        while (!ifs.eof()) {
            char buffer[1024];
            ifs.getline(buffer, 1024);

            if (buffer[0] == '#') { /* コメントの場合は何もしない */ }
            else {
                std::map<std::string, double>::iterator it = para.begin();
                while (it != para.end()) {
                    // パラメータの名前と一致したら値を読み込む
                    if (strncmp(buffer, it->first.c_str(), it->first.length()) == 0) {
                        it->second = atof(&buffer[it->first.length()]);
                        break;
                    }
                    ++it;
                }
            }
        }

        ifs.close();
        return true;
    }


public:
    std::map<std::string, double> para;
};
