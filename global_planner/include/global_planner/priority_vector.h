#ifndef PRIORITY_VECTOR_H
#define PRIORITY_VECTOR_H

#include <algorithm>
#include <vector>

template<class T>
/**
 * @brief The PriorityVector class provides template interface for creating heap-type data
 * in vector-type container.
 * Elements in that container must be pushed/poped only through declared in that class functions.
 */
class PriorityVector: public std::vector< T >
{
public:
    PriorityVector():std::vector< T >()
    {
        std::make_heap( this->begin(), this->end() );
    }

    /**
     * @brief pushes data to vector using heap features
     * @param value pushed value
     */
    void push_to_heap( T value )
    {
        this->push_back( value );
        std::push_heap( this->begin(), this->end() );
    }

    /**
     * @brief Pops element from vector using heap features
     */
    void pop_heap()
    {
        std::pop_heap( this->begin(), this->end() );
        this->pop_back();
    }
};

#endif
