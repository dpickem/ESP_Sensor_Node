/**********************************************************
 Copyright (C) 2019 Daniel Pickem (daniel.pickem@gmail.com)
 http://www.danielpickem.com - All Rights Reserved

 This software is part of the firmware running on home
 automation sensor nodes. You may use, copy, modify, merge,
 publish, distribute, sublicense, and/or sell copies of the
 Software but you must include this copyright notice and
 this permission in all copies or substantial portions of
 the software.

 Inspired by the task library written by Alan Burlison,
 alan@bleaklow.com (2011, https://github.com/fitzterra/Task)

 MIT license, all text above must be included in any
 redistribution
***********************************************************/

#ifndef task_h
#define task_h

/*
 * A simple (abstract) base class for tasks.
 *
 * NOTE: At the very minimum, the following abstract methods need
 *       to be implemented: 
 *
 *       - is_connected(),
 *       - can_run(),
 *       - setup_helper()
 *       - run_helper()
 */
class Task {
 public:
    /*
     * Optional task setup method (by default, no setup is done).
     *
     * NOTE: This method is not abstract and provides a default
     *       implementation that can be overridden in derived
     *       classes.
     *
     * NOTE: This method should be implemented in a blocking
     *       fashion, because it should only ever be called if the
     *       is_connected() method returns true, i.e. if the sensor 
     *       is connected. If called on a disconnected sensor task,
     *       this method should cause a time out and a failure to
     *       execute the main loop.
     */
    virtual bool setup() = 0;
    virtual bool setup_helper() = 0;

    // Abstract method to check whether a sensor is connected.
    virtual bool is_connected() = 0;

    // Abstract method to check whether a task can run given the
    // current time.
    virtual bool can_run(uint32_t now) = 0;

    // Abstract run method for the task.
    virtual void run(uint32_t now, bool verbose) = 0;
    virtual void run_helper() = 0;
};


/*
 * A task that is run on a periodic basis.
 */
class TimedTask : public Task {
 public:
    // Constructor for a periodically executed task.
    inline TimedTask(uint32_t next_execution_time,
                     uint32_t rate,
                     String task_name)
      : rate_(rate), name_(task_name) {
        // Initialize class variables.
        next_execution_time_ = next_execution_time; 
        is_setup_ = false;
    }

    // ------------------------------
    //        Setup methods
    // ------------------------------
    bool setup() {
      // Call the setup helper method.
      if (setup_helper()) {
        // Set the is_setup flag if the setup_helper reports
        // success.
        Serial.print("Task "); Serial.print(name_);
        Serial.println(" set up successfully.");

        is_setup_ = true;
        return true;
      }

      Serial.print("FAILED to set up task ");
      Serial.print(name_); Serial.println("...");

      return false;
    }

    // Inline setup method.
    TimedTask* setup_inline() {
      // Call the setup method from each derived class.
      setup();

      return this;
    };

    // ------------------------------
    //        Run methods
    // ------------------------------
    // Main run method that calls the tasks run_helper, which
    // implements the task's main functionality.
    inline void run(uint32_t now, bool verbose=false) {
      if (can_run(now)) {
        if(verbose) {
          Serial.print("Running task "); Serial.print(name_);
          Serial.print(" (is_setup: "); Serial.print(is_setup_);
          Serial.println(")");
        }

        // Execute task if it is set up.
        if (is_setup()) {
          // Execute run helper implemented in every derived class.
          run_helper();
        }

        // Update the time the task can be executed next.
        increment_next_execution_time(rate_);
      }
    }

    // Check if a task can be executed given the current time.
    inline bool can_run(uint32_t now) {
        return now >= next_execution_time_;
    }

    // Increment the timestamp when the task can be executed the
    // next time.
    inline void increment_next_execution_time(uint32_t dt) {
        next_execution_time_ += dt;
    }

    // Decrement the timestamp when the task can be executed the
    // next time.
    inline void decrement_next_execution_time(uint32_t dt) {
        next_execution_time_ -= dt;
    }

    // ------------------------------
    //        Getter methods
    // ------------------------------
    // Get the timestamp when the task can be executed the next
    // time.
    inline uint32_t get_next_execution_time() {
        return next_execution_time_;
    }

    inline String get_name() {
        return name_;
    }

    inline bool is_setup() {
        return is_setup_;
    }

 protected:
    String name_;
    bool is_setup_;
    uint32_t rate_;
    uint32_t next_execution_time_;
};

#endif
