from ..contracts import BusError, Call, Descriptor, TaskPort, goal_request


class NavigationModule:
    def __init__(self, tasks: TaskPort):
        self._tasks = tasks

    @property
    def descriptor(self) -> Descriptor:
        return Descriptor('navigation', ('plan', 'navigate', 'test', 'get', 'cancel', 'events'),
                          mutations=('plan', 'navigate', 'test', 'cancel'),
                          motion=('navigate',), stops=('cancel',))

    def handle(self, call: Call) -> dict:
        args = call.arguments
        if call.operation in ('plan', 'navigate', 'test'):
            return self._tasks.submit(call.operation, goal_request(args, call.operation), call.request_id)
        task = args.get('task_id')
        if not isinstance(task, str) or len(task) != 32 or any(c not in '0123456789abcdef' for c in task):
            raise BusError('INVALID_TASK_ID', 'Expected upstream task identity')
        if call.operation == 'get':
            return self._tasks.get(task)
        if call.operation == 'cancel':
            return self._tasks.cancel(task)
        after, wait = args.get('after', 0), args.get('wait', 0.)
        if type(after) is not int or after < 0 or type(wait) not in (float, int) or not 0 <= wait <= 10:
            raise BusError('INVALID_EVENT_CURSOR', 'Invalid event cursor/wait')
        return self._tasks.events(task, after, wait)
