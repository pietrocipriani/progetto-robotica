#ifndef JSON_PRINTER_HPP_INCLUDED
#define JSON_PRINTER_HPP_INCLUDED

#include "json.hpp"


namespace json {


std::ostream& operator <<(std::ostream& out, const Json& json);

std::istream& operator >>(std::istream& in, Json& json);


}

#endif /* JSON_PRINTER_HPP_INCLUDED */
