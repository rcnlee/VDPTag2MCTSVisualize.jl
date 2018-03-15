module VDPTag2MCTSVisualize

using Plots, ContinuumWorld, POMDPToolbox, MCTS, VDPTag2

export VDPTag2MCTSVis

struct VDPTag2MCTSVis
    mdp::VDPTagMDP
    s::TagState
    actions::Vector{Vector{Float64}}
    next_states::Vector{Vector{TagState}}
    ns::Vector{Int}
    radii::Vector{Vector{Float64}}
end
VDPTag2MCTSVis(mdp::VDPTagMDP, s::TagState=TagState()) = VDPTag2MCTSVis(mdp, s, Vector{Float64}[],
    Vector{TagState}[], Int[], Vector{Float64}[])

function circle(x,y,r)
    c = Shape(Plots.partialcircle(0,2π,20,r))
    translate!(c, x, y)
    c
end

@recipe function f(vis::VDPTag2MCTSVis, h::SimHistory, index::Int)
    mdp = vis.mdp
    aspect_ratio := :equal
    xlim --> (-5, 5)
    ylim --> (-5, 5)
    @series begin
        label := "actions"
        alpha := 0.25
        rng=MersenneTwister(0)
        [translate!(Shape(Plots.partialcircle(ang-r, ang+r, 20, 1.0+0.4rand(rng)-0.2)),vis.s.agent...) for (ang,r) in zip(vis.actions[index],vis.radii[index]) if r < Inf]  
    end
    @series begin
        label := "path"
        x = [s.agent[1] for s in state_hist(h)[1:end-1]]
        y = [s.agent[2] for s in state_hist(h)[1:end-1]]
        x, y
    end
    @series begin
        a = action_hist(h)[end]
        if a isa TagAction && a.look
            color := :blue
        else
            color := :red
        end
        s = state_hist(h)[end-1]
        label := "current agent position"
        pts = Plots.partialcircle(0, 2*pi, 100, mdp.tag_radius)
        x, y = Plots.unzip(pts)
        x+s.agent[1], y+s.agent[2]
    end
    @series begin
        seriestype := :scatter
        label := "current target"
        pos = state_hist(h)[end-1].target
        color --> :orange
        [pos[1]], [pos[2]]
    end
end
function MCTS.notify_listener(vis::VDPTag2MCTSVis,dsb::DSBPlanner,s,a,sp,r,snode,sanode,spnode)
    if s == vis.s
        tree = get(dsb.tree)
        sol = dsb.solver
        actions = [tree.a_labels[c] for c in tree.children[snode]]
        if isempty(vis.actions) || (actions != vis.actions[end])  #only push the deltas
            push!(vis.actions, actions)
            #push!(vis.next_states, [s+action for action in actions])
            push!(vis.ns, tree.total_n[snode])
            push!(vis.radii, [sol.r0_action/tree.total_n[snode]^sol.lambda_action for action in actions])
        end
    end
end
function MCTS.notify_listener(vis::VDPTag2MCTSVis,asb::ASBPlanner,s,a,sp,r,snode,sanode,spnode)
    if s == vis.s
        tree = get(asb.tree)
        sol = asb.solver
        actions = [tree.a_labels[c] for c in tree.children[snode]]
        if isempty(vis.actions) || (actions != vis.actions[end])  #only push the deltas
            push!(vis.actions, actions)
            #push!(vis.next_states, [s+action for action in actions])
            push!(vis.ns, tree.total_n[snode])
            push!(vis.radii, [tree.a_radius[x] for x in tree.children[snode]])
        end
    end
end
end # module
