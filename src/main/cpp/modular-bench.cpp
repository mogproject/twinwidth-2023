#include <iostream>

#include "modular/MDTree.hpp"
#include "readwrite/pace_2023.hpp"

using namespace std;

int main(int argc, char* argv[]) {
  bool print_result = false;
  bool print_sorted = false;
  vector<char const*> paths;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--print") == 0) {  // print decomposition result
      print_result = true;
    } else if (strcmp(argv[i], "--sort") == 0) {  // sort decomposition result
      print_sorted = true;
    } else {
      paths.push_back(argv[i]);
    }
  }

  if (paths.empty()) {
    printf("Usage: %s [--print] PATH\n", argv[0]);
    return 1;
  }

  bool sorted = print_result && print_sorted;

  for (auto path : paths) {
    // load graph
    auto graph = readwrite::load_pace_2023(path);

    // run algorithm
#if PROFILE_ON
    util::Profiler prof;
    auto result = modular::modular_decomposition_time(graph, sorted, &prof);
    prof.print();
#else
    auto result = modular::modular_decomposition_time(graph, sorted);
#endif

    // output result
    printf("%d\n", result.first.modular_width());
    printf("%.10f\n", result.second);
    if (print_result) printf("%s\n", result.first.to_string().c_str());
  }
  return 0;
}
