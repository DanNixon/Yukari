/** @file */

#pragma once

#include <string>
#include <map>

namespace Yukari
{
  namespace Common
  {
    class MapHelpers
    {
    public:
      template<typename K, typename V>
      static V Get(const std::map<K, V> & input, K key, V defaultValue = V())
      {
        auto it = input.find(key);
        if (it != input.end())
          return it->second;
        else
          return defaultValue;
      }
    };
  }
}