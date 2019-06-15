#pragma once

#include <stdint.h>
#include <stddef.h>
#include "common/assert.h"

namespace ul
{
template <typename T>
class list_elem
{
	public:
		list_elem(T data) : _data(data), _next(NULL) {}
		~list_elem() {};
		
		list_elem<T> *next() const { return _next; }
		
		T data() const { return _data; }
		
		void next(list_elem<T> *elem)
		{
			_next = elem;
			elem->_next = NULL;
		}
		
	private:
		T _data;
		list_elem<T> *_next;
};

template <typename T> class list
{
	public:
		list() : _head(NULL), _tail(NULL) {}
		~list()
		{
			list_elem<T> *first = _head, *to_delete;
			while(first)
			{
				to_delete = first;
				first = first->next();
				delete to_delete;
			}
		}
		
		void add(list_elem<T> *elem)
		{
			ASSERT(elem);
			
			if(!_head)
			{
				_head = elem;
				_tail = elem;
			}
			else
			{
				_tail->next(elem);
				_tail = elem;
			}
		}
		
		void remove(list_elem<T> *elem)
		{
			for(list_elem<T> *prev = NULL, *curr = _head; curr;
				prev = curr, curr = curr->next())
			{
				if(elem == curr)
				{
					if(prev)
						prev->next(curr->next());
					else
						_head = curr->next();
					
					break;
				}
			}
		}
		
		list_elem<T> *find(T data, list_elem<T> *last = NULL)
		{
			if(!last)
				last = _head;
			
			while(last)
			{
				if(last->data() == data)
					return last;
				
				last = last->next();
			}
			
			return NULL;
		}
		
		list_elem<T> *head() const { return _head; }
		
		size_t size()
		{
			size_t size = 0;
			
			for(list_elem<T> *elem = _head; elem; elem = elem->next())
				size++;
			
			return size;
		}
		
		bool is_empty() const { return !static_cast<bool>(_head); }
		
	private:
		list_elem<T> *_head;
		list_elem<T> *_tail;
};
}
