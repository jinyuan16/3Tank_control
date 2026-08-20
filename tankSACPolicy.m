function action1 = tankSACPolicy(observation1)
%#codegen

% Reinforcement Learning Toolbox
% Generated on: 20-Aug-2026 17:00:34

persistent policy;
if isempty(policy)
	policy = coder.loadRLPolicy("tankSACPolicyData.mat");
end
% evaluate the policy
action1 = getAction(policy,observation1);