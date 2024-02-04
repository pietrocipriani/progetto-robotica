#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include "json.hpp"
using namespace std;
using namespace json;

int main() {
  vector<string> tests = {
    "{}", "  {   }  ", " { a, d } ", " {  , } ",
    "{\"chia\\\"  ve1\": \"valore1\", \"chiave2\": \"valore2\", \"chiave1\": \"valoredup\" , , , , , , }",
    "{\"chiave1\": \"valore1\", \"chiave2\": \"valore2\" \"chiave1\": \"valoredup\"}",
    "[{\"title\": \"JSON – Mozilla Firefox\"}]"
  };

  for (const auto& t : tests) {
    try {
      cout << json::parse(t) << endl;
    } catch (const std::invalid_argument& e) {
      cerr << e.what() << endl;
    }
  }

  try {
    cout << json::parse(std::istreambuf_iterator<char>(cin), std::istreambuf_iterator<char>()) << endl;
  } catch (const std::invalid_argument& e) {
    cerr << e.what() << endl;
  }
}
