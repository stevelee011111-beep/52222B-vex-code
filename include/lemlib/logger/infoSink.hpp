#pragma once

#include "lemlib/logger/message.hpp"
#include "lemlib/logger/baseSink.hpp"

namespace lemlib {
/**
 * @brief Sink for sending messages to the terminal.
 *
 * This is the primary way of interacting with LemLib's logging implementation. This sink is used for printing useful
 * information to the user's terminal.
 */
class InfoSink : public BaseSink {
    public:
        /**
         * @brief Construct a new Info Sink object
         */
        InfoSink();
    private:
        /**
         * @brief Log the given message
         *
         * @param message
         */
        void sendMessage(const Message& message) override;
};
} // namespace lemlib
