from ..contracts import BusError, Call, Descriptor, WorldPort


class WorldModule:
    def __init__(self, world: WorldPort):
        self._world = world

    @property
    def descriptor(self) -> Descriptor:
        return Descriptor('world', ('waypoints',))

    def handle(self, call: Call) -> dict:
        if set(call.arguments) - {'scene'}:
            raise BusError('UNKNOWN_FIELDS', 'Only scene is accepted')
        scene = call.arguments.get('scene', 'office')
        if scene not in ('office', 'outside'):
            raise BusError('INVALID_SCENE', 'Unknown scene')
        return self._world.waypoints(scene)
