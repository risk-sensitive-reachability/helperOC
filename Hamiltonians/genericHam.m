function hamValue = genericHam(t, data, deriv, schemeData)
% function hamValue = genericHam(t, data, deriv, schemeData)

%% Input unpacking
dynSys = schemeData.dynSys;

if ~isfield(schemeData, 'uMode')
  schemeData.uMode = 'min';
end

if ~isfield(schemeData, 'dMode')
  schemeData.dMode = 'min';
end

if ~isfield(schemeData, 'tMode')
  schemeData.tMode = 'backward';
end

% Custom derivative for MIE
if isfield(schemeData, 'deriv')
  deriv = schemeData.deriv;
end

%% Optimal control and disturbance
if isfield(schemeData, 'uIn')
  u = schemeData.uIn;
else
  u = dynSys.optCtrl(t, schemeData.grid.xs, deriv, schemeData.uMode);
end

global uOpt_previous; 
global control_update_available; 
global uU_previous; 
global uL_previous;
global config; 

if t == 0
    uOpt_previous = 0; 
    uU_previous = 0; 
    uL_previous = 0; 
    control_update_available = 0; 
end

if t >= control_update_available

    disp('hamiltonian: new optimal control'); 
    uOpt_previous = u; 
    % time update in genericPartial 
   % control_update_available = control_update_available + config.dt; 
else 
   % disp('hamiltonian: previous optimal control'); 
end

u = uOpt_previous; 

if isfield(schemeData, 'dIn')
  d = schemeData.dIn;
else
  d = dynSys.optDstb(t, schemeData.grid.xs, deriv, schemeData.dMode);
end

hamValue = 0;
%% MIE
if isfield(schemeData, 'side')
  if strcmp(schemeData.side, 'lower')
%     if schemeData.dissComp
%       hamValue = hamValue - schemeData.dc;
%     end
%     
    TIderiv = -1;
  elseif strcmp(schemeData.side, 'upper')
%     if schemeData.dissComp
%       hamValue = hamValue + schemeData.dc;
%     end
    
%     hamValue = -hamValue;
%     deriv{1} = -deriv{1};
% %     if schemeData.trueMIEDeriv
% %       deriv{2} = -deriv{2};
% %     end
    TIderiv = 1;
  else
    error('Side of an MIE function must be upper or lower!')
  end
  
%   if ~schemeData.trueMIEDeriv
%     deriv(2) = computeGradients(schemeData.grid, data);
%   end
end

%% Plug optimal control into dynamics to compute Hamiltonian
dx = dynSys.dynamics(t, schemeData.grid.xs, u, d);
for i = 1:dynSys.nx
  hamValue = hamValue + deriv{i}.*dx{i};
end

if isprop(dynSys, 'TIdim') && ~isempty(dynSys.TIdim)
  TIdx = dynSys.TIdyn(t, schemeData.grid.xs, u, d);
  hamValue = hamValue + TIderiv*TIdx{1};
end

%% Negate hamValue if backward reachable set
if strcmp(schemeData.tMode, 'backward')
  hamValue = -hamValue;
end

if isfield(schemeData, 'side')
  if strcmp(schemeData.side, 'upper')
    hamValue = -hamValue;
  end
end
end