// OSX: "clang++ -std=c++11 ./random_unit.cpp -o random"
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <string>
int main(int init_argc, char **init_argv)
{
  using namespace std;
  auto count = 10;
  {
    auto argc = init_argc;
    auto argv = init_argv;
    auto error = false;
    ++argv;
    --argc;
    while (argc) {
      if (string("--count") == string(*argv)) {
        ++argv;
        --argc;
        if (!argc) {
          error = true;
          break;
        }
        try {
          count = stoi(*argv);
        } catch (...) {
          cerr << "'" << *argv << "': expected a number" << endl;
          error = true;
        }
      }
      --argc;
      --argv;
    }
    if (error) {
      cerr << "USAGE: " << init_argv[0] << " --count [n]" << endl;
    }
  }
  uint32_t min_number = numeric_limits<uint32_t>::max();
  uint32_t max_number = numeric_limits<uint32_t>::min();
  cout << "#pragma once" << endl;
  cout << "/* Generated by " << init_argv[0] << " */" << endl;
  cout << "internal_symbol u32 const random_number_table[] = {" << endl;
  mt19937 engine;
  while (count) {
    auto per_line = 4;
    cout << "    ";
    while (per_line--) {
      if (count) {
        uint32_t y = engine();
        min_number = min(min_number, y);
        max_number = max(max_number, y);
        cout << "0x" << hex << right << uppercase << y;
        cout << ", ";
        --count;
      }
    }
    cout << endl;
  }
  cout << "};" << endl;
  cout << "internal_symbol u32 const min_random_number = 0x" << hex << uppercase
       << min_number << ";" << endl;
  cout << "internal_symbol u32 const max_random_number = 0x" << hex << uppercase
       << max_number << ";" << endl;
}