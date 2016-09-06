#API for publishing and subscribing to message topics
export Publisher, Subscriber, publish

"""
    Publisher{T}(topic; kwargs...)
    Publisher(topic, T; kwargs...)

Create an object to publish messages of type `T` on a topic. Keyword arguments are directly passed
to rospy.
"""
type Publisher{MsgType<:MsgT}
    o::PyObject

    function Publisher(topic::AbstractString; kwargs...)
        @debug("Creating <$(string(MsgType))> publisher on topic: '$topic'")
        rospycls = _get_rospy_class(MsgType)
        return new(__rospy__[:Publisher](ascii(topic), rospycls; kwargs...))
    end
end
Publisher{MsgType<:MsgT}(topic::AbstractString, ::Type{MsgType}; kwargs...) =
    Publisher{MsgType}(ascii(topic); kwargs...)

"""
    publish(p::Publisher{T}, msg::T)

Publish `msg` on `p`, a `Publisher` with matching message type.
"""
function publish{MsgType<:MsgT}(p::Publisher{MsgType}, msg::MsgType)
    pycall(p.o["publish"], PyAny, convert(PyObject, msg))
end

if VERSION >= v"0.5.0-dev+3692" #callbacks are broken

type Subscriber{T}
end
Subscriber(args...) = error(
"""Subscribing to a topic is currently broken on julia v0.5 and above. See
https://github.com/jdlangs/RobotOS.jl/issues/15 for ongoing efforts to fix this.""")

else #callbacks not broken

@doc """
    Subscriber{T}(topic, callback, cb_args=(); kwargs...)
    Subscriber(topic, T, callback, cb_args=(); kwargs...)

Create a subscription to a topic with a callback to use when a message is received, which can be any
callable type. Keyword arguments are directly passed to rospy.
""" ->
type Subscriber{MsgType<:MsgT}
    o::PyObject
    callback

    function Subscriber(topic::AbstractString, cb, cb_args::Tuple=(); kwargs...)
        @debug("Creating <$(string(MsgType))> subscriber on topic: '$topic'")
        rospycls = _get_rospy_class(MsgType)
        jl_cb(msg::PyObject) = cb(convert(MsgType, msg), cb_args...)
        return new(
            __rospy__[:Subscriber](ascii(topic), rospycls, jl_cb; kwargs...),
            jl_cb
        )
    end
end

Subscriber{MsgType<:MsgT}(
    topic::AbstractString,
    ::Type{MsgType},
    cb,
    cb_args::Tuple=();
    kwargs...
) = Subscriber{MsgType}(ascii(topic), cb, cb_args; kwargs...)

end #version check
