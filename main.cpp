#include <iostream>
#include <fstream>
#include "Map.h"
#include "LowLevel.h"
#include "CBS.h"
#include "windows.h"
#include <chrono>
#include <psapi.h>

void print_memory_usage();
void printMemoryUsage();
//void logMemoryAndTime(double elapsed);

int main() {
    highLevelCBS CBS;
    CBS.readInput();

    auto start = std::chrono::high_resolution_clock::now();
    CBS.RunCBS();
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = end - start;
    std::cout << "runtime: " << elapsed.count() << " s" << std::endl;

    print_memory_usage();
    //logMemoryAndTime(elapsed.count());
}


void print_memory_usage() {
    PROCESS_MEMORY_COUNTERS_EX pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
        SIZE_T physMemUsedByMe = pmc.WorkingSetSize;
        SIZE_T physMemPeakUsedByMe = pmc.PeakWorkingSetSize;
        SIZE_T virtualMemUsedByMe = pmc.PrivateUsage;

        std::cout << "Memory Usage:" << std::endl;
        std::cout << "Physical Memory Used: " << physMemUsedByMe / 1024 << " KB" << std::endl;
        std::cout << "Peak Physical Memory Used: " << physMemPeakUsedByMe / 1024 << " KB" << std::endl;
        std::cout << "Virtual Memory Used: " << virtualMemUsedByMe / 1024 << " KB" << std::endl;
    } else {
        std::cerr << "Failed to retrieve memory usage information" << std::endl;
    }
}

//void logMemoryAndTime(double elapsed) {
//    std::ofstream logFile("memory_and_time_log.txt", std::ios::app);
//    if (logFile.is_open()) {
//        PROCESS_MEMORY_COUNTERS pmc;
//        if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
//            logFile << "当前内存使用量: " << pmc.WorkingSetSize / 1024 << " KB" << std::endl;
//            logFile << "最大内存使用量: " << pmc.PeakWorkingSetSize / 1024 << " KB" << std::endl;
//        } else {
//            logFile << "无法获取内存信息" << std::endl;
//        }
//        logFile << "运行时间: " << elapsed << " 秒" << std::endl;
//        logFile << "------------------------------" << std::endl;
//        logFile.close();
//    } else {
//        std::cerr << "无法打开日志文件" << std::endl;
//    }
//}
