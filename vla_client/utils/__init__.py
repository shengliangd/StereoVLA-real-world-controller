def input_typed(prompt: str, tp: type):
    if tp == bool:
        tp_fn = lambda x: {'y': True, 'n': False}[x]
    else:
        tp_fn = tp
    while True:
        try:
            return tp_fn(input(prompt))
        except:
            print('invalid input')


def input_choice(prompt: str, choices: list):
    print('please choose from:')
    for i, choice in enumerate(choices):
        print(f'{i}: {choice}')
    while True:
        try:
            choice = int(input(prompt))
            assert choice in range(len(choices))
            return choices[choice]
        except KeyboardInterrupt:
            raise
        except:
            print('invalid input')

import yaml
import pickle
import os


class TrajectoryRecorder:
    """Not thread safe!"""
    def __init__(self, folder):
        self.folder = folder
        try:
            os.makedirs(folder)
        except:
            print(f'output folder {folder} exists')
            exit(1)
        self.steps = []
        self.metadata = {}

        self.terminated = False

    def submit_metadata(self, metadata):
        self.metadata = metadata

    def submit_step(self, step):
        self.steps.append(step)

    def ask_about_result(self):
        return
        print('please carefully input the result:')
        result = dict([
            ('success', input_typed('success? (y/n): ', bool)),
            ('grounding', input_typed('grounding? (y/n): ', bool)),
            ('moved', input_typed('moved the object? (y/n): ', bool)),
            ('trials', input_typed('trials(int): ', int)),
        ])
        return result

    def finish(self):
        assert not self.terminated
        self.terminated = True
        with open(f'{self.folder}/record.pickle', 'wb+') as f:
            pickle.dump({
                'metadata': self.metadata,
                'steps': self.steps,
            }, f)
        print(f'saved to {self.folder}')
        with open(f'{self.folder}/metric.yml', 'w+') as f:
            yaml.dump(self.ask_about_result(), f)
