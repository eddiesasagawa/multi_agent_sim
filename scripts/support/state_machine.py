'''
Generic State Machine implementation
'''

class State(object):
    def __init__(self):
        '''
        On entry code goes here
        '''
        pass
    
    def run(self):
        '''
        Main execution step goes here
        '''
        pass

    def on_exit(self):
        '''
        On exit code goes here
        '''
        pass

class Condition(object):
    def __init__(self, fcn, next_state):
        self.fcn = fcn
        self.next_state = next_state

    def evaluate(self):
        return self.next_state if self.fcn() else None

class Transition(object):
    def __init__(self, source_state_cls, conditions_list):
        self.source_state_cls = source_state_cls
        self.conditions_list = conditions_list

    def next(self, current_state):
        for condition in self.conditions_list:
            result = condition.evaluate()
            if result is not None:
                current_state.on_exit() # close out the current state
                return result() # <- instantiate the next state
        else:
            return current_state

class StateMachine(object):
    '''
    EXAMPLE:
    sm = StateMachine([
        Transition(S1, [
            Condition(lambda: path.reached_goal, S2)
        ]),
        Transition(S2, [
            Condition(lambda: rotated > 6.28, S1)
        ])
    ])
    '''
    def __init__(self, transition_list, init_state_cls):
        self.transition_list = transition_list
        self.transition_table = {}
        for transition in transition_list:
            self.transition_table[transition.source_state_cls] = transition

        self.init_state_cls = init_state_cls
        self.current_state = init_state_cls()

    def run(self):
        self.transition_table[type(self.current_state)].next(self.current_state)
        self.current_state.run()