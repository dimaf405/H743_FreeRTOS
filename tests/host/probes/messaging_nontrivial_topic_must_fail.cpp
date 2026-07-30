#include "Dima/middleware/messaging/topic.hpp"

#include <string>

struct NonTriviallyCopyableMessage {
    std::string payload;
};

// This translation unit must never compile: Topic itself, rather than merely a
// convenience trait, must reject a non-trivially-copyable payload.
dima::middleware::messaging::Topic<NonTriviallyCopyableMessage> rejected_topic;
