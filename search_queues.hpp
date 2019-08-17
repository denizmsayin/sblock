#ifndef __SEARCH_QUEUES_HPP__
#define __SEARCH_QUEUES_HPP__

#include <queue>
#include <stack>
#include <vector>

template <class T, class C>
class Queue {
private:
    std::queue<T> q;

public:
    Queue() : q() {}

    bool empty() const { return q.empty(); }

    void push(const T &e) { q.push(e); }

    const T &top() const { return q.front(); }

    void pop() { q.pop(); }
};

template <class T, class C>
class Stack {
private:
    std::stack<T> s;

public:
    Stack() : s() {}

    bool empty() const { return s.empty(); }

    void push(const T &e) { s.push(e); }

    const T &top() const { return s.top(); }

    void pop() { s.pop(); }
};

template <class T, class C>
class PriorityQueue {
private:
    std::priority_queue<T, std::vector<T>, C> pq;

public:
    PriorityQueue() : pq() {}

    bool empty() const { return pq.empty(); }

    void push(const T &e) { pq.push(e); }

    const T &top() const { pq.top(); }

    void pop() { pq.pop(); }
};




#endif
