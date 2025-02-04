#include <iostream>
#include <array>

class LeftQueue {
private:
    static const int MAX_SIZE = 5000;  // 队列长度上限
    std::array<std::array<float, 9>, MAX_SIZE> queue;
    int front;
    int rear;
    int current_size;

public:
    LeftQueue() : front(0), rear(0), current_size(0) {}

    // 判断队列是否为空
    bool isEmpty() const {
        return current_size == 0;
    }

    // 判断队列是否已满
    bool isFull() const {
        return current_size == MAX_SIZE;
    }

    // 尾部添加一个长度为9的数组
    bool enqueue(const std::array<float, 9>& arr) {
        if (isFull()) {
            std::cout << "Queue is full, cannot enqueue!" << std::endl;
            return false;
        }
        queue[rear] = arr;
        rear = (rear + 1) % MAX_SIZE;
        current_size++;
        return true;
    }

    // 头部拉取一个长度为9的数组
    bool dequeue(std::array<float, 9>& arr) {
        if (isEmpty()) {
            std::cout << "Queue is empty, cannot dequeue!" << std::endl;
            return false;
        }
        arr = queue[front];
        front = (front + 1) % MAX_SIZE;
        current_size--;
        return true;
    }

    // 获取当前队列长度
    int getSize() const {
        return current_size;
    }

    // 清空队列
    void clear() {
        front = 0;
        rear = 0;
        current_size = 0;
    }
};

int main() {
    LeftQueue leftQueue;
    std::array<float, 9> arr1 = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9};
    std::array<float, 9> arr2 = {9.9, 8.8, 7.7, 6.6, 5.5, 4.4, 3.3, 2.2, 1.1};

    std::cout << "Queue size: " << leftQueue.getSize() << std::endl;
    
    // 添加数组到队列
    leftQueue.enqueue(arr1);
    leftQueue.enqueue(arr2);

    std::cout << "Queue size: " << leftQueue.getSize() << std::endl;

    // 从队列拉取数组
    std::array<float, 9> arr_out;
    if (leftQueue.dequeue(arr_out)) {
        std::cout << "Dequeued array: ";
        for (float f : arr_out) {
            std::cout << f << " ";
        }
        std::cout << std::endl;
    }

    // 清空队列
    leftQueue.clear();
    std::cout << "Queue size after clearing: " << leftQueue.getSize() << std::endl;

    return 0;
}
