#include "ParamLayer.h"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

ParamLayer::ParamLayer(ParamLayer *parent)
: _parent(parent)
{}
