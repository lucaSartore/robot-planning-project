// source: https://stackoverflow.com/questions/7110301/generic-hash-for-tuples-in-unordered-map-unordered-set
#pragma once
#include <tuple>
using namespace  std;

struct hash_duplet {
    template <class T1, class T2>
    size_t operator()(const tuple<T1, T2> &x) const {
        auto h0 = hash<T1>{}(std::get<0>(x));
        auto h1 = hash<T2>{}(std::get<1>(x));
        return h0 ^ (h1 << 1);
    }
};

struct hash_triplet {
    template <class T1, class T2, class T3>
    size_t operator()(const tuple<T1, T2, T3> &x) const {
        auto h0 = hash<T1>{}(std::get<0>(x));
        auto h1 = hash<T2>{}(std::get<1>(x));
        auto h2 = hash<T3>{}(std::get<2>(x));
        return h0 ^ (h1 << 1) ^ (h2 << 2);
    }
};

struct hash_quadruplet {
    template <class T1, class T2, class T3, class T4>
    size_t operator()( const tuple<T1, T2, T3, T4>& x) const {
        auto h0 = hash<T1>{}(std::get<0>(x));
        auto h1 = hash<T2>{}(std::get<1>(x));
        auto h2 = hash<T3>{}(std::get<2>(x));
        auto h3 = hash<T4>{}(std::get<3>(x));
        return h0 ^ (h1 << 1) ^ (h2 << 2) ^ (h3 << 3);
    }
};



