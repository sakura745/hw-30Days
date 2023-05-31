#include <iostream>
#include <vector>
#include <algorithm>

template<typename T>
void quickSort(typename std::vector<T>::iterator begin, typename std::vector<T>::iterator end) {
    if (end - begin < 2) return;
    auto pivot = *begin;
    auto left = begin;
    auto right = end - 1;
    while (left <= right) {
        while (*left < pivot)
            left++;
        while (*right > pivot)
            right--;
        if (left <= right) {
            std::swap(*left, *right);
            left++;
            right--;
        }
    }
    quickSort<T>(begin, right + 1);
    quickSort<T>(left, end);
}

int main() {
    std::vector<int> v = {3, 1, 4, 1, 5, 9, 2, 6, 5, 3, 5};
    quickSort<int>(v.begin(), v.end());
    for (auto i : v) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    return 0;
}


