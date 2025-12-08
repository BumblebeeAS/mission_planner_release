#!/usr/bin/env python3

import py_trees
import py_trees.console as console


def stop_on_success_or_failure(tree):
    if tree.root.status == py_trees.common.Status.SUCCESS:
        console.loginfo(console.green + "completed one execution" + console.reset)
        raise SystemExit
    elif tree.root.status == py_trees.common.Status.FAILURE:
        console.loginfo(console.red + "stopped on failure" + console.reset)
        raise SystemExit
