#Test publish and subscribe ability
#works alongside echonode.py
#typegeneration.jl must be run first

using geometry_msgs.msg

const Nmsgs = 10
const rate = 20. #Hz
const msgs = PoseStamped[]
const refs = Array(Vector3, Nmsgs)
const t0 = to_nsec(get_rostime())

for i=1:Nmsgs
    refs[i] = Vector3(rand(3)...)
end

const ros_pub = Publisher("vectors", Vector3, queue_size = 10)
rossleep(Duration(3.0))

function publish_messages(pubobj, msgs, rate_hz)
    const r = Rate(rate_hz)
    for msg in msgs
        publish(pubobj, msg)
        rossleep(r)
    end
    rossleep(Duration(1.0))
end

function pose_cb(msg::PoseStamped, msgs::Vector{PoseStamped})
    mtime = to_nsec(msg.header.stamp) - t0
    mtime > 0 && println("Message received, time: ",mtime," nanoseconds")
    if msg.header.stamp.secs > 1.0
        push!(msgs, msg)
        println("Got message #",msg.header.seq)
    end
end
pose_cb(PoseStamped(), msgs) #warm up run

if VERSION >= v"0.5.0-dev+3692" #callbacks are broken

warn("Not testing subscriber!")

publish_messages(ros_pub, refs, 20.0)
rossleep(Duration(5.0))
Nreceived = get_param("/num_received_messages")
@test Nreceived == length(refs)
set_param("/num_received_messages", 0)

else #callbacks not broken

const ros_sub = Subscriber("poses", PoseStamped, pose_cb, (msgs,), queue_size = 10)

#First message doesn't go out for some reason
publish(ros_pub, Vector3(1.1,2.2,3.3))
rossleep(Duration(1.0))
publish_messages(ros_pub, refs, 20.0)
rossleep(Duration(1.0))

println("Received ",length(msgs)," / ",Nmsgs)

@test length(msgs) == Nmsgs
for i=1:Nmsgs
    @test_approx_eq msgs[i].pose.position.x refs[i].x
    @test_approx_eq msgs[i].pose.position.y refs[i].y
    @test_approx_eq msgs[i].pose.position.z refs[i].z
end
empty!(msgs)

end #version check
