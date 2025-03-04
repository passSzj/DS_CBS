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

