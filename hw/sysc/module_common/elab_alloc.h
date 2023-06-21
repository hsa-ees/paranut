//
// Created by ripopov on 12/18/17.
//

#ifndef ELAB_ALLOC_H
#define ELAB_ALLOC_H

#include <cstdint>
#include <typeinfo>



namespace sc_core
{

/// Allocate singular object dynamically and store information for SVC elaboration
template<class T, class... Args>
T* sc_new(Args&&... args)
{
    T* rptr = ::new T(std::forward<Args>(args)...);
    return rptr;
}

/// Allocate array dynamically and store information for SVC elaboration
template<class T>
T* sc_new_array(size_t n)
{
    T* rptr = ::new T[n];
    return rptr;
}

}

#endif // ELAB_ALLOC_H