#API for calling/creating services. Syntax is practically identical to rospy.

type ServiceProxy{SrvType <: ServiceDefinition}
    o::PyObject
    
    function ServiceProxy(name::String; kwargs...)
        rospycls = _get_rospy_class(SrvType)
        new(__rospy__.ServiceProxy(name, rospycls; kwargs...))
    end
end
function ServiceProxy{SrvType<:ServiceDefinition}(
    name::String, 
    srv::Type{SrvType}; 
    kwargs...
)
    ServiceProxy{SrvType}(name; kwargs...)
end

function Base.call{SrvType <: ServiceDefinition}(
    srv::ServiceProxy{SrvType}, 
    req::SrvT
)
    if ! isa(req, _srv_reqtype(SrvType))
        throw(ArgumentError(
            string("Incorrect service request type: ",typeof(req))))
    end
    pyresp = pycall(srv.o, PyObject, convert(PyObject, req))
    resp = convert(_srv_resptype(SrvType), pyresp)
    resp
end

type Service{SrvType <: ServiceDefinition}
    o::PyObject
    jl_handler::Function

    function Service(name::String, handler::Function; kwargs...)
        rospycls = _get_rospy_class(SrvType)
        ReqType = _srv_reqtype(SrvType)
        jl_handler(req::PyObject) = handler(convert(ReqType,req))
        try
            new(__rospy__.Service(name, rospycls, jl_handler; kwargs...), 
                jl_handler
            )
        catch err
            if isa(err, PyCall.PyError)
                error("Problem during service creation: $(err.val[:args][1])")
            else
                rethrow(err)
            end
        end
    end
end
function Service{SrvType<:ServiceDefinition}(
    name::String,
    srv::Type{SrvType},
    handler::Function;
    kwargs...
)
    Service{SrvType}(name, handler; kwargs...)
end

function wait_for_service(service::String; kwargs...)
    try
        __rospy__.wait_for_service(service; kwargs...)
    catch ex
        error("Timeout exceeded waiting for service '$service'")
    end
end