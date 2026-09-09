#include "atomic_transaction.h"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace px4 {

AtomicTransaction::AtomicTransaction() noexcept
{ dima::parameters::detail::transaction_begin(); }

AtomicTransaction::~AtomicTransaction()
{ dima::parameters::detail::transaction_end(); }

void AtomicTransaction::lock() noexcept
{ dima::parameters::detail::transaction_lock(); }

void AtomicTransaction::unlock() noexcept
{ dima::parameters::detail::transaction_unlock(); }

} // namespace px4
