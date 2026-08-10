"""Single-ship layer: decides for one platform. Purely cyber.

Binds down to the subsystem layer and to peers here; never to a resource,
and never upward. No component in this layer may hold a port of type
truth.* -- it is two layers above anything entitled to read ground truth.
"""
