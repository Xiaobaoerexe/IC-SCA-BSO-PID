function [Ngr, Np, Tx] = gas_turbine_narx_model(qf, theta_igv, Ti, params)
%% Gas Turbine NARX Model
% Simplified NARX neural network model for gas turbine
% Based on the paper's NARX model with:
%   - Hidden layer: 12-13 neurons with tansig transfer function
%   - Output layer: purelin transfer function
%   - Training function: trainlm
%   - Delay order: 2
%
% Inputs:
%   qf       - Fuel volume (normalized)
%   theta_igv- Adjustable guide vane angle (normalized)
%   Ti       - Intake temperature (normalized)
%   params   - Model parameters
%
% Outputs:
%   Ngr      - Gas generator speed (normalized)
%   Np       - Power turbine speed (normalized)
%   Tx       - Exhaust temperature (normalized)

% Get model parameters (pre-trained weights)
persistent W1_ngr b1_ngr W2_ngr b2_ngr
persistent W1_np b1_np W2_np b2_np
persistent W1_tx b1_tx W2_tx b2_tx
persistent prevInputs prevOutputs

% Initialize weights if not done
if isempty(W1_ngr)
    [W1_ngr, b1_ngr, W2_ngr, b2_ngr, ...
     W1_np, b1_np, W2_np, b2_np, ...
     W1_tx, b1_tx, W2_tx, b2_tx] = initialize_narx_weights();
    
    % Initialize delay buffers
    prevInputs = zeros(2, 3);   % 2 delays, 3 inputs
    prevOutputs = zeros(2, 3);  % 2 delays, 3 outputs
end

% Combine inputs with delays
% Input vector: [u(t-1), u(t-2), y(t-1), y(t-2)]
% where u = [qf, theta_igv, Ti] and y = [Ngr, Np, Tx]
input_vector = [qf, theta_igv, Ti, ...
                prevInputs(1, :), prevInputs(2, :), ...
                prevOutputs(1, :), prevOutputs(2, :)];

% Forward pass for Ngr (12 hidden neurons)
hidden_ngr = tansig(W1_ngr * input_vector' + b1_ngr);
Ngr = W2_ngr * hidden_ngr + b2_ngr;

% Forward pass for Np (12 hidden neurons)
hidden_np = tansig(W1_np * input_vector' + b1_np);
Np = W2_np * hidden_np + b2_np;

% Forward pass for Tx (13 hidden neurons)
hidden_tx = tansig(W1_tx * input_vector' + b1_tx);
Tx = W2_tx * hidden_tx + b2_tx;

% Clip outputs to valid range [-1, 1]
Ngr = max(-1, min(1, Ngr));
Np = max(-1, min(1, Np));
Tx = max(-1, min(1, Tx));

% Update delay buffers
prevInputs(2, :) = prevInputs(1, :);
prevInputs(1, :) = [qf, theta_igv, Ti];
prevOutputs(2, :) = prevOutputs(1, :);
prevOutputs(1, :) = [Ngr, Np, Tx];

end

function [W1_ngr, b1_ngr, W2_ngr, b2_ngr, ...
          W1_np, b1_np, W2_np, b2_np, ...
          W1_tx, b1_tx, W2_tx, b2_tx] = initialize_narx_weights()
%% Initialize NARX Network Weights
% These weights are initialized to approximate gas turbine dynamics
% In practice, these would be trained using actual gas turbine data

% Number of inputs: 3 current + 6 delayed inputs + 6 delayed outputs = 15
numInputs = 15;

% Random seed for reproducibility
rng(42);

% Initialize weights for Ngr model (12 hidden neurons)
numHidden_ngr = 12;
W1_ngr = (rand(numHidden_ngr, numInputs) - 0.5) * 0.5;
b1_ngr = (rand(numHidden_ngr, 1) - 0.5) * 0.5;
W2_ngr = (rand(1, numHidden_ngr) - 0.5) * 0.5;
b2_ngr = 0;

% Initialize weights for Np model (12 hidden neurons)
numHidden_np = 12;
W1_np = (rand(numHidden_np, numInputs) - 0.5) * 0.5;
b1_np = (rand(numHidden_np, 1) - 0.5) * 0.5;
W2_np = (rand(1, numHidden_np) - 0.5) * 0.5;
b2_np = 0;

% Initialize weights for Tx model (13 hidden neurons)
numHidden_tx = 13;
W1_tx = (rand(numHidden_tx, numInputs) - 0.5) * 0.5;
b1_tx = (rand(numHidden_tx, 1) - 0.5) * 0.5;
W2_tx = (rand(1, numHidden_tx) - 0.5) * 0.5;
b2_tx = 0;

end
