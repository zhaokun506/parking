#include "common/matplot/matplotlibcpp.h"
#include <map>
#include <vector>
using namespace std;
namespace plt = matplotlibcpp;

int main() {

  vector<int> x, y;
  x = {0, 5, 10};
  y = {0, 5, 0};
  // plt::plot({1, 3, 2, 4});
  map<string, string> keywords;
  keywords["color"] = "grey";
  plt::fill(x, y, keywords);

  vector<int> x1, y1;
  x1 = {10, 15, 20};
  y1 = {0, 5, 0};
  // plt::plot({1, 3, 2, 4});
  map<string, string> keywords1;
  keywords1["color"] = "red";
  plt::fill(x1, y1, keywords1);

  plt::show();
}
