import time

from itertools import takewhile

from .meta import direct_path
from .smoothing import smooth_path, smooth_path_old
from .rrt import TreeNode, configs
from .utils import irange, argmin, RRT_ITERATIONS, RRT_RESTARTS, RRT_SMOOTHING, INF, elapsed_time, \
    negate


def asymmetric_extend(q1, q2, extend_fn, backward=False):
    if backward:
        return reversed(list(extend_fn(q2, q1)))
    return extend_fn(q1, q2)

def extend_towards(tree, target, distance_fn, extend_fn, collision_fn, swap, tree_frequency, return_collide, out_p):
    last = argmin(lambda n: distance_fn(n.config, target), tree)
    extend = list(asymmetric_extend(last.config, target, extend_fn, swap))
    safe = []
    if return_collide:
        for item in extend:
            result = collision_fn(item)
            if isinstance(result, bool) and not result:
                safe.append(item)
            if not isinstance(result, bool):
                # Save the output to the out_p
                out_p = result
    else:
        safe = list(takewhile(negate(collision_fn), extend))
    for i, q in enumerate(safe):
        if (i % tree_frequency == 0) or (i == len(safe) - 1):
            last = TreeNode(q, parent=last)
            tree.append(last)
    success = len(extend) == len(safe)
    return last, success

def rrt_connect(q1, q2, distance_fn, sample_fn, extend_fn, collision_fn,
                iterations=RRT_ITERATIONS, tree_frequency=1, max_time=INF,
                max_iterations=None, return_collide=False):
    start_time = time.time()
    assert tree_frequency >= 1
    if return_collide:
        a = collision_fn(q1)
        b = collision_fn(q2)
        if not isinstance(a, bool):
            return False, a
        if not isinstance(b, bool):
            return False, b 
    else:
        if collision_fn(q1) or collision_fn(q2):
            return None
    nodes1, nodes2 = [TreeNode(q1)], [TreeNode(q2)]
    if max_iterations is not None:
        iterations = max_iterations
    out_p = None
    for iteration in irange(iterations):
        # print(iteration)
        if max_time <= elapsed_time(start_time):
            break
        swap = len(nodes1) > len(nodes2)
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        last1, _ = extend_towards(tree1, sample_fn(), distance_fn, extend_fn, collision_fn,
                                  swap, tree_frequency, return_collide, out_p)
        last2, success = extend_towards(tree2, last1.config, distance_fn, extend_fn, collision_fn,
                                        not swap, tree_frequency, return_collide, out_p)
        if success:
            path1, path2 = last1.retrace(), last2.retrace()
            if swap:
                path1, path2 = path2, path1
            #print('{} iterations, {} nodes'.format(iteration, len(nodes1) + len(nodes2)))
            return configs(path1[:-1] + path2[::-1])
    if return_collide:
        return False, out_p
    return None

#################################################################

def birrt(q1, q2, distance, sample, extend, collision,
          restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING, max_time=INF, return_collide=False, **kwargs):
    # TODO: move to the meta class
    start_time = time.time()
    if return_collide:
        a = collision(q1)
        b = collision(q2)
        # if not isinstance(a, bool):
        #     return False, a
        # if not isinstance(b, bool):
        #     return False, b 
    else:
        if collision(q1) or collision(q2):
            return None
 
    path = direct_path(q1, q2, extend, collision)
    if path is not None:
        return path
    if max_time != INF:
        while elapsed_time(start_time) < max_time:
            if max_time <= elapsed_time(start_time):
                break
            path = rrt_connect(q1, q2, distance, sample, extend, collision,
                            max_time=max_time - elapsed_time(start_time), return_collide=return_collide, **kwargs)
            if path is not None and not isinstance(path, tuple):
                #print('{} attempts'.format(attempt))
                if smooth is None:
                    return path
                #return smooth_path_old(path, extend, collision, iterations=smooth)
                return smooth_path(path, extend, collision, distance_fn=distance, iterations=smooth,
                                max_time=max_time - elapsed_time(start_time))
    else:
        for attempt in irange(restarts + 1):
            # print("attempt", attempt)
            # TODO: use the restart wrapper
            if max_time <= elapsed_time(start_time):
                break
            path = rrt_connect(q1, q2, distance, sample, extend, collision,
                            max_time=max_time - elapsed_time(start_time), return_collide=return_collide, **kwargs)
            if path is not None and not isinstance(path, tuple):
                #print('{} attempts'.format(attempt))
                if smooth is None:
                    return path
                #return smooth_path_old(path, extend, collision, iterations=smooth)
                return smooth_path(path, extend, collision, distance_fn=distance, iterations=smooth,
                                max_time=max_time - elapsed_time(start_time))
        
    if path is not None and isinstance(path, tuple):
        return path
    return None
