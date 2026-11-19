"""Stage boundary types. Mutable state exists only inside a single invocation."""
from enum import Enum


class Flow(Enum):
    NEXT = 'next'
    CONTINUE = 'continue'
    BREAK = 'break'


class ScopeFrame:
    __slots__ = ('_values',)

    def __init__(self, values):
        allowed = type(self).__annotations__
        if set(values) - set(allowed):
            raise TypeError('Undeclared stage state')
        object.__setattr__(self, '_values', dict(values))

    @classmethod
    def capture(cls, scope):
        return cls({key: scope[key] for key in cls.__annotations__ if key in scope})

    def has(self, name):
        return name in self._values

    def __getattr__(self, name):
        if name not in type(self).__annotations__:
            raise AttributeError(name)
        try:
            return self._values[name]
        except KeyError:
            raise UnboundLocalError('Stage input is undefined: '+name) from None

    def __setattr__(self, name, value):
        if name not in type(self).__annotations__:
            raise AttributeError('Undeclared stage state: '+name)
        self._values[name] = value
