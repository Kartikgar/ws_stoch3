/*
 * file: state_machine.h
 *
 * Created: 29 Oct, 2021
 * Author : Aditya Sagi
 */

#pragma once

#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <map>


/***** FUNCTION ************/
/*
 * This implementation is of an event triggered state machine.
 * To obtain a polled state machine, it can be tiggered by timer events.
 *
 * In this state machine an action occurs in the state and not during the 
 * transition. Each state has three distinct functions that are defined:
 * 1) init
 * 2) run
 * 3) exit
 *
 * The init function is called once when the state is entered. When a state is 
 * entered, the run function is also called after the init function.
 *
 * The run function is called when the state machine is triggered without defining any 
 * transition event for that particular state. Note that the run function will not be
 * called when the state machine is triggered with a valid transition event.
 *
 * The exit function is called when a transition to the next state occurs.
 *
 * When the state machine is started, the first state that is registered with the state
 * machine will be entered. 
 */

/****** USAGE ***************/
/*
 * 1) Define the states (as classes) by inheriting the 
 * abstract class 'State'.
 *
 * 2) In the application instantiate the state machine
 * and the states.
 *
 * 3) Register all the states with the state machine
 * using the addState function.
 *
 * 4) Register all the transitions between states by using
 * the addTransition function.
 *
 * 5) Start the state machine by invoking the start function.
 *
 * 6) Trigger the state machine when an event occurs (or on 
 * timer event) using the trigger function.
 *
 *
 * Example code:
 *
 * #include "state_machine.h"
 *
 * class State1: public State
 * {
 * ...
 * };
 *
 * class State2: public State
 * {
 * ...
 * };
 *
 * int main()
 * {
 *  StateMachine sm;
 *  State1 state1;
 *  State2 state2;
 *
 *  sm.addState(&state1);
 *  sm.addState(&state2);
 *
 *  sm.addTransition("State1", "event12", "State2");
 *  sm.addTransition("State2", "event21", "State1");
 *
 *  sm.start();
 *
 *  sm.trigger();
 *
 *  sm.trigger("event12");
 *  sm.trigger("event21");
 * }
 */

namespace utils
{

  /* 
   * An abstract class to represent the state in the state machine.
   * All states being derived must inherit this abstract class
   * as public.
   */
  class State
  {
    public:
      virtual std::string getName() = 0;

      /*
       * Initial function that will be called when the state is entered.
       *
       * \param[in] prev_state: Name of the state from which the current state
       *                  is being entered
       *
       */
      virtual void init(std::string prev_state) = 0;

      /*
       * Runs every time the state machine is triggered
       *
       */
      virtual void run() = 0;

      /* 
       * Function that is called when the state is being exited.
       *
       * \param[in] next_state: Name of the state to which the
       *                        state machine transfers control.
       */
      virtual void exit(std::string next_state) = 0;
  };

  /* 
   * A structure to represent the transitions between states
   */
  typedef struct
  {
    /*
     * The name of the state from which the transition is being defined.
     */
    std::string current_state;

    /*
     * A map of the event (names) and the state to which the state machine 
     * should transfer control if that event occurs.
     */
    std::map<std::string, std::string> event_state_map;
  } Transition;

  /*
   * The state machine class which handles the interaction between 
   * states and the transfer of control to the different states.
   */
  class StateMachine
  {
    private:
      /* 
       * A list of all the states in the state machine.
       */
      std::vector<State*> states_;

      /*
       * A list of all the transitions being defined between states.
       * One element will contain the details of all the transition from 
       * one state.
       *
       * Each element of the list will contain:
       * [state, [{event1, next_state1}, {event2, next_state2},..., {eventN, next_stateN}]
       */
      std::vector<Transition> transitions_;

      /*
       * A pointer to the current state that has control in the state machine.
       */
      State* current_state;

      /* A flag to indicate if the state machine has been started. 
       * Note: The state machine must be stated with an explicit
       * call to the start function.
       */
      bool sm_started = false;

    public:

      /*
       * A function to add states to the state machine
       *
       * \param[in] new_state: A pointer to the state to be added.
       *
       * Returns true on success and false on failure.
       *
       * Note: All the states being added must have the same scope as the state
       * machine since pointers are being used. A state pointer going out of scope
       * before the state machine is closed can lead to runtime errors.
       */
      bool addState(State* new_state)
      {
        std::string name = new_state->getName();
        for(auto & state : states_)
        {
          if(name == state->getName())
          {
            printf("Cannot add the state: %s. A state with the same name already exists.\n", name.c_str());
            return false;
          }
        }
        states_.push_back(new_state);
        return true;

      }

      /* 
       * A function to add transitions between two states.
       *
       * \param[in] current_state: Name of the state from which the transition is being defined
       * \param[in] event: Name of the event which leads to the transition
       * \param[in] next_state: Name of the state to which the control is transferred
       *
       * Returns true on success and false on failure.  
       */
      bool addTransition(std::string current_state, std::string event, std::string next_state)
      {
        bool found = false;
        for(auto state : states_)
        {
          if(state->getName() == current_state)
          {
            found = true;
            break;
          }
        }

        if(!found)
        {
          printf("A state with the name: %s, is not registered with the state machine.\n", current_state.c_str());
          return false;
        }

        // Look for the transitions of the same state
        for(auto& transition : transitions_)
        {
          if(transition.current_state == current_state)
          {
            if(transition.event_state_map.count(event))
            {  
              printf("A transition for the event, %s, from state, %s, already exists. Cannot add the transition.\n", event.c_str(), current_state.c_str());
              return false;
            }
            else
            {
              transition.event_state_map.insert(std::pair<std::string, std::string>(event, next_state));
              return true;
            }
          }
        }

        // If an existing transition is not found, then add it
        {
          Transition transition;
          transition.current_state = current_state;
          transition.event_state_map.insert(std::pair<std::string, std::string>(event, next_state));
          transitions_.push_back(transition);
        }
        return true;

      }

      /*
       * A function to start the state machine.
       *
       * Note: This function must be explcitly called to start the state machine.
       * This function must be called after registering all the states and the transitions.
       */ 
      void start()
      {
        if(sm_started)
          return;

        current_state = states_[0];
        current_state->init("");
        current_state->run();
        sm_started = true;
        return;

      }

      /* 
       * A function to stop the state machine.
       */
      void stop()
      {
        current_state->exit("");
        current_state = 0;
        sm_started = false;
        return;
      }

      /*
       * A function to get the current state.
       *
       * \returns the current state
       */
      std::string getState()
      {
        return current_state->getName();
      }

      /*
       * A function to trigger the state machine.
       */ 
      void trigger()
      {
        if(!sm_started)
        {
          printf("State machine not started.\n");
          return;
        }

        current_state->run();
        return;

      }

      /*
       * A function to trigger the state machine when an event occurs.
       *
       * \param[in] event: Name of event that occurs.
       */
      void trigger(std::string event)
      {
        State* next_state;
        Transition* transition;

        if(!sm_started)
        {
          printf("State machine not started.\n");
          return;
        }

        // If event is a valid transition event for the current
        // state, then exit the current state and init the next state
        if(transition = findTransition(current_state->getName()))
        {
          auto it = transition->event_state_map.find(event);
          if(it != transition->event_state_map.end())
          {
            if(next_state = findState(it->second))
            {
              current_state->exit(next_state->getName());
              next_state->init(current_state->getName());
              next_state->run();
              current_state = next_state;
              return;
            }
            else
            {
              printf("Error: could not find target state for the state transition.");
            }
          }
        }
        return;
      }

      /**** Utility functions ****/

      /*
       * A function to list the names of all the registered states.
       *
       * Returns a vector of names of all the registered states.
       */
      std::vector<std::string> getStateNames()
      {
        std::vector<std::string> state_list;

        for(auto & state : states_)
          state_list.push_back(state->getName());

        return state_list;
      }


      /*
       * A function to print all the transitions that 
       * are defined in the state machine.
       */
      void printTransition()
      {
        for(auto transition : transitions_)
        {
          std::cout << transition.current_state << " :" << std::endl;
          for(const auto& pair : transition.event_state_map)
          {
            std::cout << "\t" << pair.first << " : " << pair.second << std::endl;
          }
        }
        return;
      }
      /*
       * A utility function to find the pointer of a
       * state given its name.
       *
       * \param[in] name: Name of the state to search for.
       *
       * Returns the pointer to the state if it is found,
       * else returns 0.
       */
      State* findState(std::string name)
      {
        for(auto state : states_)
        {
          if(state->getName() == name)
            return state;
        }
        return 0;
      }
      /*
       * A function to find the transition of the given state.
       *
       * \param[in] name: Name of the state for which the 
       *                  transition is required.
       *
       * Returns the pointer to the transition if found,
       * else returns 0.
       */ 
      Transition* findTransition(std::string name)
      {
        for(auto& transition : transitions_)
        {
          if(transition.current_state == current_state->getName())
          {
            return &transition;
          }
        }
        return 0; 
      }
  };
}
