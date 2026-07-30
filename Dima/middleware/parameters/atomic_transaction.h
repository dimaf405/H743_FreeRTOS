/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the conditions in the upstream BSD-3-Clause license are met.
 *
 ****************************************************************************/
#pragma once

namespace dima::parameters::detail {
void transaction_begin() noexcept;
void transaction_end() noexcept;
void transaction_lock() noexcept;
void transaction_unlock() noexcept;
}

namespace px4 {
class AtomicTransaction {
public:
    AtomicTransaction() noexcept { dima::parameters::detail::transaction_begin(); }
    ~AtomicTransaction() { dima::parameters::detail::transaction_end(); }
    void lock() noexcept { dima::parameters::detail::transaction_lock(); }
    void unlock() noexcept { dima::parameters::detail::transaction_unlock(); }
    AtomicTransaction(const AtomicTransaction &) = delete;
    AtomicTransaction &operator=(const AtomicTransaction &) = delete;
};
}
using AtomicTransaction = px4::AtomicTransaction;
// Upstream path: src/lib/parameters/atomic_transaction.h @ d6f12ad1
