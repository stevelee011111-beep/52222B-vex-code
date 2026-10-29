#pragma once

#include <cstdint>

namespace lemlib {
class Timer {
    public:
        /**
         * @brief Construct a new Timer
         *
         * @note the timer will start counting down as soon as it is created
         * @note the timer constructor is non-blocking so code after it will be executed immediately
         * @note if the timer is constructed in a global scope, its behavior is undefined. You can
         *       call set() before using the timer if you absolutely need to construct it in a global scope
         *
         * @param time how long to wait, in milliseconds
         */
        Timer(uint32_t time = 120000);

        /**
         * @brief Get the amount of time the timer was set to
         *
         * @return uint32_t time, in milliseconds
         */
        uint32_t getTimeSet();

        /**
         * @brief Get the amount of time left on the timer
         *
         * @return uint32_t time in milliseconds
         */
        uint32_t getTimeLeft();

        /**
         * @brief Get the amount of time passed on the timer
         *
         * @return uint32_t time in milliseconds
         */
        uint32_t getTimePassed();

        /**
         * @brief Get whether the timer is done or not
         *
         * @return true the timer is done
         * @return false the timer is not done
         */
        bool isDone();

        /**
         * @brief Get whether the timer is paused or not
         *
         * @return true the timer is paused
         * @return false the timer is not paused
         */
        bool isPaused();

        /**
         * @brief Set the amount of time the timer should count down. Resets the timer
         *
         * @param time time in milliseconds
         */
        void set(uint32_t time);

        /**
         * @brief reset the timer
         */
        void reset();

        /**
         * @brief pause the timer
         */
        void pause();

        /**
         * @brief resume the timer
         */
        void resume();

        /**
         * @brief wait until the timer is done
         */
        void waitUntilDone();

    private:
        uint32_t period;
        uint32_t lastTime;
        uint32_t timeWaited = 0;
        bool paused = false;
};
} // namespace lemlib