// source: https://stackoverflow.com/questions/7110301/generic-hash-for-tuples-in-unordered-map-unordered-set
#pragma once
#include <tuple>
using namespace  std;

struct hash_duplet {
    template <class T1, class T2>
    size_t operator()(const tuple<T1, T2> &x) const {
        return get<0>(x) ^ (get<1>(x) << 1);
    }
};

struct hash_triplet {
    template <class T1, class T2, class T3>
    size_t operator()(const tuple<T1, T2, T3> &x) const {
        return get<0>(x) ^ (get<1>(x) << 1) ^ (get<2>(x) << 2);
    }
};

struct hash_quadruplet {
    template <class T1, class T2, class T3, class T4>
    size_t operator()( const tuple<T1, T2, T3, T4>& x) const {
        return get<0>(x) ^ (get<1>(x) << 1) ^ (get<2>(x) << 2) ^ (get<3>(x) << 3);
    }
};



