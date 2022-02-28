#ifndef __OBJECTS_H__
#define __OBJECTS_H__
#include "Headers/Common.hpp"
#include "Headers/Utils.hpp"
#include "Headers/Objects.hpp"
#include "Headers/Publishers.hpp"
#include "Headers/PointClouds.hpp"
#include "Headers/Accumulator.hpp"
#include "Headers/Compensator.hpp"
#include "Headers/Localizator.hpp"
#include "Headers/Mapper.hpp"
#endif

extern struct Params Config;

template class Buffer<IMU>;
template class Buffer<Point>;
template class Buffer<State>;

// class Buffer {
    // public:
        template <typename ContentType>
        Buffer<ContentType>::Buffer() {}

        template <typename ContentType>
        void Buffer<ContentType>::push(const ContentType& cnt) {
            this->content.push_front(cnt);
        }

        template <typename ContentType>
        void Buffer<ContentType>::pop_front() {
            this->content.pop_front();
        }

        template <typename ContentType>
        void Buffer<ContentType>::pop_back() {
            this->content.pop_back();
        }
        
        template <typename ContentType>
        ContentType Buffer<ContentType>::front() {
            return this->content.front();
        }

        template <typename ContentType>
        ContentType Buffer<ContentType>::back() {
            return this->content.back();
        }
        
        template <typename ContentType>
        bool Buffer<ContentType>::empty() {
            return this->content.empty();
        }

        template <typename ContentType>
        int Buffer<ContentType>::size() {
            return this->content.size();
        }

        template <typename ContentType>
        void Buffer<ContentType>::clear() {
            return this->content.clear();
        }

        template <typename ContentType>
        void Buffer<ContentType>::clear(TimeType t) {
            auto* buffer = &this->content;
            while (buffer->size() > 0 and t >= buffer->back().time)
                this->pop_back();
        }