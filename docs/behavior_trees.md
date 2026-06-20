# Mission Planner conventions

Mission Planner 2 builds on
[py_trees](https://py-trees.readthedocs.io/) and
[py_trees_ros](https://py-trees-ros.readthedocs.io/).

## Blackboard

- Namespace keys by vehicle and task.
- Pass keys and frame names into shared builders instead of relying on
  task-specific globals.
- Build action and service requests on the blackboard when their values depend
  on runtime perception or mission state.

## Shared builders

Keep shared search and alignment builders vehicle-agnostic. Inject movement
behaviour classes and frame names rather than importing a vehicle-specific
`goto` implementation.

## Cleanup and recovery

- Stop perception and clustering on every exit path.
- Keep retry counts finite.
- Use explicit fallback branches for reduced-capability behaviour.
- Place cleanup outside a retry body when it must run after both success and
  failure.

## Frames

State the frame of every mission goal explicitly. Keep vehicle, camera, tool,
and target frames separate, and use frame-aware helpers instead of hand-written
transform math.

When a tool rather than the vehicle centre must reach a target, pass the tool's
anchor frame to the movement behaviour.
