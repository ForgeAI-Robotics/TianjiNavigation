// Radius-graph connectivity for immutable point arrays. No robot interfaces.
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <vector>

extern "C" int tianji_components(std::size_t count, std::size_t edge_count,
                                const std::int64_t* edges, std::int64_t* labels) {
    try {
        std::vector<std::size_t> parent(count), rank(count, 0);
        std::iota(parent.begin(), parent.end(), 0);
        auto root = [&](std::size_t node) {
            while (parent[node] != node) {
                parent[node] = parent[parent[node]];
                node = parent[node];
            }
            return node;
        };
        for (std::size_t i = 0; i < edge_count; ++i) {
            auto a = edges[2 * i], b = edges[2 * i + 1];
            if (a < 0 || b < 0 || static_cast<std::size_t>(a) >= count ||
                static_cast<std::size_t>(b) >= count) return 1;
            auto left = root(static_cast<std::size_t>(a));
            auto right = root(static_cast<std::size_t>(b));
            if (left == right) continue;
            if (rank[left] < rank[right]) std::swap(left, right);
            parent[right] = left;
            if (rank[left] == rank[right]) ++rank[left];
        }
        std::vector<std::int64_t> numbers(count, -1);
        std::int64_t next = 0;
        for (std::size_t i = 0; i < count; ++i) {
            auto component = root(i);
            if (numbers[component] < 0) numbers[component] = next++;
            labels[i] = numbers[component];
        }
        return 0;
    } catch (...) {
        return 2;
    }
}
