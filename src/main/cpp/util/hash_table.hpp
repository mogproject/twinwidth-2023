#pragma once
#include <cstdint>

namespace util {
std::size_t const HASH_TABLE_SIZE = 1 << 20;
extern uint64_t HASH_TABLE[HASH_TABLE_SIZE];

void initialize_hash_table();

uint64_t get_hash(std::size_t index);
}  // namespace util
