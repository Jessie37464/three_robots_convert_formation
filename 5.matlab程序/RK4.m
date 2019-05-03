function x= RK4( Fun, x, h, t, varargin )

%-------------------------------------------------------------------------------
%   Fourth order Runge-Kutta. Called function is of the form:
%
%   Fun(x,t,varargin)
%
%   Accepts optional arguments that are passed through to Fun. Time is also
%   optional.                    接受可选参数，它被传递到函数。时间也是可选的。
%   This version is streamlined and designed to take advantage of MATLAB's
%   function handles feature (MATLAB versions 6 and up only). Passing a 
%   string function name (version 5) will also work but is slower.
% 
%-------------------------------------------------------------------------------
%   Form:
%   x = RK4( Fun, x, h, t, varargin )     varargin 实际输入的参量
%-------------------------------------------------------------------------------
%
%   ------
%   Inputs
%   ------
%   Fun                Function    Fun(x,{t,...})
%   x                  State (column vector)      状态(列向量)
%   h                  Independent variable step   独立的变步长
%   t                  Current time                当前时间
%   p1...              Optional arguments          可选参数；
%
%   -------
%   Outputs
%   -------
%   x                  Updated state
%
%-------------------------------------------------------------------------------

%-------------------------------------------------------------------------------
%   Copyright 1993-1994, 2006 Princeton Satellite Systems, Inc.
%   All rights reserved.
%-------------------------------------------------------------------------------

if( nargin < 4 )  %函数输入变量数小于4
  t = [];
end

ho2 = 0.5*h;

if ~isempty(t)    %如果t不是空矩阵
  k1  = feval( Fun, x, t, varargin{:} );
  k2  = feval( Fun, x + ho2*k1, t+ho2, varargin{:} );
  k3  = feval( Fun, x + ho2*k2, t+ho2, varargin{:} );
  k4  = feval( Fun, x + h*k3, t+h, varargin{:} );
else
  k1  = feval( Fun, x, varargin{:} );
  k2  = feval( Fun, x + ho2*k1, varargin{:} );
  k3  = feval( Fun, x + ho2*k2, varargin{:} );
  k4  = feval( Fun, x + h*k3, varargin{:} );
end


x   = x + h*(k1 + 2*(k2+k3) + k4)/6;


%--------------------------------------
% SZU file version information
%--------------------------------------
% $Date: 2015-01-02 $

