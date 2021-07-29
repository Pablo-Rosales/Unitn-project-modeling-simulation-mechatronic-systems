%
% Matlab code for the Course:
%
%     Modelling and Simulation of Mechatronics System
%
% by
% Enrico Bertolazzi
% Dipartimento di Ingegneria Industriale
% Universita` degli Studi di Trento
% email: enrico.bertolazzi@unitn.it
%
classdef PegasusImplicitDAE < DAE3baseClassImplicit
 
  properties (SetAccess = protected, Hidden = true)
  
  end

  methods 
    %
    %  Abstract functions defining an index-3 DAE with some derivatives
    %
    %  q' = v
    %  M(t,p) v' + Phi_p(t,q)^T lambda = gforce( t, q, v )
    %  Phi(t,q) = 0
    %
    %  d Phi(t,q) / dt     = A(t,q,v)
    %  d^2 Phi(t,q) / dt^2 = Phi_q(t,q) v - b(t,q,v)
    %
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function jac__Mass = M( self, t, pos)
      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      
      
      % evaluate function
      t1 = cos(theta4);
      t2 = t1 ^ 2;
      t3 = 0.2600e2 * t2;
      t4 = sin(theta4);
      t6 = -0.6240e2 * t4 - 0.789360e3;
      t8 = 0.526240e3 * t4;
      t10 = cos(theta3);
      t11 = t10 ^ 2;
      jac__1_1 = t11 * (-t3 + t1 * t6 - t8 - 0.1798345873e5) - 0.94640e2 * t1 + 0.1852240e4 * t4 + 0.2943142587e6;
      jac__1_2 = t10 * (0.624e2 * t2 - 0.26e2 * t4 * t1 - 0.39988e3 * t1 - 0.83668e3 * t4 - 0.5478293e5);
      t21 = sin(theta3);
      jac__1_3 = (-0.4732e2 * t1 + 0.92612e3 * t4 + 0.6933333333e2) * t21;
      jac__2_1 = jac__1_2;
      jac__2_2 = t3 - t1 * t6 + t8 + 0.2285921120e5;
      jac__3_1 = jac__1_3;
      jac__3_3 = 0.6933333333e2;

      % store on output
      jac__Mass = zeros(3,3);
      jac__Mass(1,1) = jac__1_1;
      jac__Mass(1,2) = jac__1_2;
      jac__Mass(1,3) = jac__1_3;
      jac__Mass(2,1) = jac__2_1;
      jac__Mass(2,2) = jac__2_2;
      jac__Mass(3,1) = jac__3_1;
      jac__Mass(3,3) = jac__3_3;
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function Phi(t,q)
    function res__Phi = Phi( self, t, pos )
      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      
      % evaluate function
      t1 = t * pi;
      res__1 = -t1 / 3 + theta2;
      res__2 = -0.11e2 / 0.30e2 * t1 + theta3;

      % store on output
      res__Phi = zeros(2,1);
      res__Phi(1) = res__1;
      res__Phi(2) = res__2;
      
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial Phi(t,q) / \partial t
    function jac__DPhiDt = Phi_t(self, t, pos)
      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      
      % evaluate function

      % store on output
      jac__DPhiDt = zeros(2,1);
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial Phi(t,q) / \partial q
    function jac__Phi_q = Phi_q(self, t, pos)
      
        % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      
      % evaluate function
      jac__1_1 = 1;
      jac__2_2 = 1;

      % store on output
      jac__Phi_q = zeros(2,3);
      jac__Phi_q(1,1) = jac__1_1;
      jac__Phi_q(2,2) = jac__2_2;
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial Phi_q(t,q)*v / \partial q
    function jac__PhiV_q = PhiV_q( self, t, pos)
     
      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);

      % evaluate function

      % store on output
      jac__PhiV_q = zeros(2,3);
      
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial Phi_q(t,q)^T*lambda / \partial q
    function jac__PhiL_q = PhiL_q( self, t, pos )
      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      
      % evaluate function
      jac__1_1 = lambda1;
      jac__2_2 = lambda1;
      jac__3_1 = lambda2;
      jac__4_2 = lambda2;

      % store on output
      jac__PhiL_q = zeros(4,3);
      jac__PhiL_q(1,1) = jac__1_1;
      jac__PhiL_q(2,2) = jac__2_2;
      jac__PhiL_q(3,1) = jac__3_1;
      jac__PhiL_q(4,2) = jac__4_2;
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial ( M(t,q) v_dot ) / \partial q
    function jac__W_q = W_q( t, pos, vel )

      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      
      theta2__dot = vel(1);
      theta3__dot = vel(2);
      theta4__dot = vel(3);
      
      % evaluate function
      t1 = cos(theta4);
      t2 = t1 ^ 2;
      t4 = sin(theta4);
      t5 = t4 * t1;
      t10 = cos(theta3);
      t12 = sin(theta3);
      jac__1_2 = 2 * 0 * t12 * t10 * (0.2600e2 * t2 + 0.6240e2 * t5 + 0.789360e3 * t1 + 0.526240e3 * t4 + 0.1798345873e5) + 0 * (-0.6240e2 * t2 + 520 * t1 * (0.5e-1 * t4 + 0.769e0) + 0.836680e3 * t4 + 0.5478293000e5) * t12 + 0 * (-0.47320e2 * t1 + 0.926120e3 * t4 + 0.6933333333e2) * t10;
      t32 = 0 * t4;
      t40 = t10 ^ 2;
      t44 = 0 * t4;
      t54 = 0 * t12;
      jac__1_3 = t40 * (-0.12480e3 * t2 * 0 + t1 * (0.5200e2 * t32 - 0.526240e3 * 0) + 0.6240e2 * 0 + 0.789360e3 * t32) + t10 * (-0.5200e2 * 0 * t2 + t1 * (-0.12480e3 * t44 - 0.836680e3 * 0) + 0.2600e2 * 0 + 0.399880e3 * t44) + t1 * (0.1852240e4 * 0 + 0.926120e3 * t54) + t4 * (0.94640e2 * 0 + 0.47320e2 * t54);
      jac__2_2 = t12 * (-0.624e2 * t2 + 0.26e2 * t5 + 0.39988e3 * t1 + 0.83668e3 * t4 + 0.5478293e5) * 0;
      jac__2_3 = t2 * (-0.5200e2 * 0 * t10 + 0.12480e3 * 0) + t1 * (t10 * (-0.12480e3 * t32 - 0.8366800000e3 * 0) - 0.5200e2 * t44 + 0.526240e3 * 0) + t10 * (0.2600e2 * 0 + 0.399880e3 * t32) - 0.6240e2 * 0 - 0.789360e3 * t44;
      jac__3_2 = t10 * (-0.4732e2 * t1 + 0.92612e3 * t4 + 0.6933333333e2) * 0;
      jac__3_3 = t12 * (0.4732e2 * t4 + 0.92612e3 * t1) * 0;

      % store on output
      jac__W_q = zeros(3,3);
      jac__W_q(1,2) = jac__1_2;
      jac__W_q(1,3) = jac__1_3;
      jac__W_q(2,2) = jac__2_2;
      jac__W_q(2,3) = jac__2_3;
      jac__W_q(3,2) = jac__3_2;
      jac__W_q(3,3) = jac__3_3;
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function jac__f = gforce(self, t, pos, vel, lambda )

      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      theta2__dot = vel(1);
      theta3__dot = vel(2);
      theta4__dot = vel(3);
      
      lambda1 = lambda(1);
      lambda2 = lambda(2); 
      
      % evaluate function
      t1 = cos(theta3);
      t2 = t1 ^ 2;
      t6 = theta3__dot * theta4__dot;
      t8 = theta2__dot * theta3__dot;
      t9 = sin(theta3);
      t14 = theta3__dot ^ 2;
      t18 = cos(theta4);
      t19 = t18 ^ 2;
      t21 = theta2__dot * theta4__dot;
      t23 = sin(theta4);
      t25 = theta4__dot * theta2__dot * t23;
      t29 = t23 * t8;
      t30 = 0.12480e3 * t29;
      t36 = theta4__dot * theta3__dot * t23;
      t41 = 0.2600e2 * t23 * t14;
      t42 = cos(theta2);
      t43 = 0.1081024847e4 * t42;
      t45 = theta4__dot ^ 2;
      t50 = sin(theta2);
      t51 = 0.7206832312e3 * t50;
      t66 = 0.7206832312e3 * t42;
      t75 = 0.1081024847e4 * t50;
      jac__1_1 = t19 * (0.12480e3 * theta4__dot * t2 * theta2__dot + t1 * (0.5200e2 * t6 - 0.5200e2 * t9 * t8) + 0.6240e2 * t9 * t14) + t18 * (t2 * (0.526240e3 * t21 - 0.5200e2 * t25) + t1 * (t9 * (-t30 - 0.1578720e4 * t8) + 0.884000e3 * t6 + 0.12480e3 * t36) + t9 * (-t41 - t43 - 0.399880e3 * t14 - 0.926120e3 * t45) - 0.1852240e4 * t21 - t51) + t2 * (-0.789360e3 * t25 - 0.6240e2 * t21) + t1 * (t9 * (-0.1052480e4 * t29 - 0.3596691746e5 * t8) - 0.1326000e4 * t36 - 0.9533333333e2 * t6) + t9 * (t23 * (-t66 - 0.836680e3 * t14 - 0.47320e2 * t45) - 0.8927297215e5 * t42 - 0.5478293000e5 * t14) + t23 * (t75 - 0.94640e2 * t21) + 0.5334164654e6 * t50 - 0.1e1 * lambda1;
      t82 = theta2__dot ^ 2;
      t88 = t23 * t82;
      jac__2_1 = t1 * (t19 * (0.5200000000e2 * t21 + 0.2600e2 * t9 * t82) + t18 * (t9 * (0.789360e3 * t82 + 0.6240e2 * t88) + 0.7893600000e3 * t21 - t75 + 0.12480e3 * t25) + t9 * (0.1798345873e5 * t82 + 0.526240e3 * t88) + t23 * (0.5262400000e3 * t21 - t51) + 0.4333333335e2 * t21 - 0.8927297215e5 * t50) - 0.12480e3 * theta4__dot * theta3__dot * t19 + t18 * (0.1081024847e4 * t9 + 0.5200e2 * t36 - 0.526240e3 * t6) + t9 * (0.7206832312e3 * t23 + 0.8927297215e5) - 0.1e1 * lambda2 + 0.6240e2 * t6 + 0.789360e3 * t36;
      t138 = t9 * t50;
      jac__3_1 = t19 * (-0.5200e2 * theta2__dot * t1 * theta3__dot + 0.6240e2 * t14 - 0.6240e2 * t82 * t2) + t18 * (t2 * (0.2600e2 * t88 - 0.263120e3 * t82) + t1 * (-0.7893600000e3 * t8 - 0.7206832312e3 - t30) - t43 + 0.263120e3 * t14 + 0.926120e3 * t82 - t41 - 0.7206832312e3 * t138) + t2 * (0.3120e2 * t82 + 0.394680e3 * t88) + t1 * (t23 * (-0.5262400000e3 * t8 + 0.1081024847e4) - 0.4333333333e2 * t8) + t23 * (-0.394680e3 * t14 + 0.1081024847e4 * t138 + 0.47320e2 * t82 - t66) - 0.2000e4 * theta4__dot - 0.10000e5 * theta4 - 0.3120e2 * t14;

      % store on output
      jac__f = zeros(3,1);
      jac__f(1,1) = jac__1_1;
      jac__f(2,1) = jac__2_1;
      jac__f(3,1) = jac__3_1;
      
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial f( t, q, v ) / \partial q
    function jac__gforce_q = gforce_q( self, t, pos, vel )
      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      theta2_dot = vel(1);
      theta3_dot = vel(2);
      theta4_dot = vel(3);

      % evaluate function
      t1 = sin(theta4);
      t3 = cos(theta4);
      t6 = cos(theta2);
      t8 = 0.1081024847e4 * t3;
      t11 = sin(theta3);
      t13 = sin(theta2);
      jac__1_1 = t6 * (0.1081024847e4 * t1 - 0.7206832308e3 * t3 + 0.5334164654e6) + t13 * t11 * (t8 + 0.7206832308e3 * t1 + 0.8927297215e5);
      t15 = theta2_dot * theta3_dot;
      t16 = t3 ^ 2;
      t19 = t1 * t15;
      t27 = cos(theta3);
      t28 = t27 ^ 2;
      t31 = theta4_dot * t11 * theta2_dot;
      t33 = theta3_dot ^ 2;
      t38 = 0.2600e2 * t33;
      t42 = 0.399880e3 * t33;
      t43 = theta4_dot ^ 2;
      t44 = 0.926120e3 * t43;
      t45 = 0.1081024847e4 * t6;
      t49 = 0.47320e2 * t43;
      t50 = 0.7206832312e3 * t6;
      t51 = 0.836680e3 * t33;
      t60 = theta4_dot * t11 * theta3_dot;
      t62 = 0.5200e2 * t15;
      t66 = 0.12480e3 * t15;
      t73 = 0.1052480e4 * t15;
      jac__1_2 = t28 * (-0.10400e3 * t16 * t15 + t3 * (-0.24960e3 * t19 - 0.3157440e4 * t15) - 0.2104960e4 * t19 - 0.7193383492e5 * t15) + t27 * (t16 * (-0.24960e3 * t31 + 0.6240e2 * t33) + t3 * (t1 * (0.10400e3 * t31 - t38) - 0.1052480e4 * t31 - t42 - t44 - t45) + t1 * (0.1578720e4 * t31 - t49 - t50 - t51) + 0.12480e3 * t31 - 0.8927297215e5 * t6 - 0.5478293000e5 * t33) + t16 * (-0.5200e2 * t60 + t62) + t3 * (t1 * (-0.12480e3 * t60 + t66) + 0.1578720e4 * t15 - 0.884000e3 * t60) + t1 * (t73 + 0.1326000e4 * t60) + 0.9533333333e2 * t60 + 0.3596691746e5 * t15;
      t84 = theta3_dot * theta4_dot;
      t92 = theta2_dot * theta4_dot;
      t93 = t1 * t92;
      t98 = 0.10400e3 * t19;
      t101 = t1 * t84;
      t107 = 0.12480e3 * t1 * t33;
      t110 = 0.1081024847e4 * t13;
      t130 = 0.7206832312e3 * t13;
      jac__1_3 = t16 * (-0.10400e3 * theta4_dot * t28 * theta2_dot + t27 * (-0.24960e3 * t11 * t15 + 0.24960e3 * t84) - 0.5200e2 * t11 * t33) + t3 * (t28 * (-0.24960e3 * t93 - 0.789360e3 * t92) + t27 * (t11 * (-t73 + t98) - 0.10400e3 * t101 - 0.1326000e4 * t84) + t11 * (-t107 - t50 - t49 - t51) + t110 - 0.94640e2 * t92) + t28 * (0.5200e2 * t92 - 0.526240e3 * t93) + t27 * (t11 * (0.1578720e4 * t19 + t66) - 0.12480e3 * t84 - 0.884000e3 * t101) + t11 * (t1 * (t45 + t44 + t42) + t38) + t1 * (0.1852240e4 * t92 + t130);
      jac__2_1 = t6 * t27 * (-t8 - 0.7206832313e3 * t1 - 0.8927297216e5);
      t137 = theta2_dot ^ 2;
      t139 = 0.5200e2 * t28 * t137;
      t140 = 0.2600e2 * t137;
      t143 = t1 * t137;
      t144 = 0.12480e3 * t143;
      t149 = 0.7893600000e3 * t92;
      t164 = 0.5262400000e3 * t92;
      jac__2_2 = t16 * (-0.5200000000e2 * t31 + t139 - t140) + t3 * (t28 * (t144 + 0.1578720e4 * t137) + 0.1081024847e4 * t27 + t11 * (-t149 + t110 - 0.12480e3 * t93) - 0.6240e2 * t143 - 0.789360e3 * t137) + t28 * (0.3596691746e5 * t137 + 0.1052480e4 * t143) + t27 * (0.7206832312e3 * t1 + 0.8927297215e5) + t11 * (t1 * (-t164 + t130) + 0.8927297215e5 * t13 - 0.4333333335e2 * t92) - 0.526240e3 * t143 - 0.1798345873e5 * t137;
      t173 = t11 * t137;
      jac__2_3 = t27 * (t16 * (0.12480e3 * t173 + 0.24960e3 * t92) + t3 * (t1 * (-0.1040000000e3 * t92 - 0.5200e2 * t173) - t130 + t164 + 0.526240e3 * t173) + t1 * (t110 - t149 - 0.789360e3 * t173) - 0.6240e2 * t173 - 0.12480e3 * t92) + 0.10400e3 * t16 * t84 + t3 * (0.24960e3 * t101 + 0.789360e3 * t84 + 0.7206832312e3 * t11) + t1 * (0.526240e3 * t84 - 0.1081024847e4 * t11) - 0.5200e2 * t84;
      t204 = t11 * t6;
      jac__3_1 = t1 * (0.1081024847e4 * t204 + t130) + t3 * (-0.7206832312e3 * t204 + t110);
      t219 = 0.7893600000e3 * t15;
      t227 = 0.5262400000e3 * t15;
      t233 = t27 * t13;
      jac__3_2 = t11 * (t16 * (0.12480e3 * t137 * t27 + t62) + t3 * (t27 * (-0.5200e2 * t143 + 0.526240e3 * t137) + t219 + 0.12480e3 * t19 + 0.7206832312e3) + t27 * (-0.789360e3 * t143 - 0.6240e2 * t137) + t1 * (t227 - 0.1081024847e4) + 0.4333333333e2 * t15) - 0.7206832312e3 * t3 * t233 + 0.1081024847e4 * t1 * t233;
      t251 = t11 * t13;
      jac__3_3 = t16 * (-0.5200e2 * t33 + t139 - 0.24960e3 * theta3_dot * t27 * theta2_dot) + t3 * (t28 * (t144 + 0.394680e3 * t137) + t27 * (0.1081024847e4 - t227 + t98) - 0.394680e3 * t33 + 0.47320e2 * t137 + 0.1081024847e4 * t251 - t107 - t50) + t28 * (-t140 + 0.263120e3 * t143) + t27 * (t1 * (t219 + 0.7206832312e3) + t66) + t1 * (-0.263120e3 * t33 + 0.7206832312e3 * t251 - 0.926120e3 * t137 + t45) + t38 - 0.10000e5;


      % store on output
      jac__gforce_q = zeros(3,3);
      jac__gforce_q(1,1) = jac__1_1;
      jac__gforce_q(1,2) = jac__1_2;
      jac__gforce_q(1,3) = jac__1_3;
      jac__gforce_q(2,1) = jac__2_1;
      jac__gforce_q(2,2) = jac__2_2;
      jac__gforce_q(2,3) = jac__2_3;
      jac__gforce_q(3,1) = jac__3_1;
      jac__gforce_q(3,2) = jac__3_2;
      jac__gforce_q(3,3) = jac__3_3;
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial f( t, q, v ) / \partial v
    function jac__gforce_v = gforce_v( self, t, pos, vel)
      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      theta2_dot = vel(1);
      theta3_dot = vel(2);
      theta4_dot = vel(3);

      % evaluate function
      t1 = cos(theta4);
      t2 = t1 ^ 2;
      t3 = t2 * theta4_dot;
      t5 = sin(theta4);
      t6 = t5 * theta4_dot;
      t14 = cos(theta3);
      t15 = t14 ^ 2;
      t17 = sin(theta3);
      t21 = theta3_dot * t5;
      t22 = 0.12480e3 * t21;
      t34 = t1 * theta4_dot;
      jac__1_1 = t15 * (0.12480e3 * t3 + t1 * (-0.5200e2 * t6 + 0.526240e3 * theta4_dot) - 0.6240e2 * theta4_dot - 0.789360e3 * t6) + t14 * (-0.5200e2 * t2 * t17 * theta3_dot + t1 * t17 * (-t22 - 0.1578720e4 * theta3_dot) + t17 * (-0.1052480e4 * t21 - 0.3596691746e5 * theta3_dot)) - 0.94640e2 * t6 - 0.1852240e4 * t34;
      t36 = t2 * theta2_dot;
      t38 = t5 * theta2_dot;
      t39 = 0.12480e3 * t38;
      t56 = t2 * theta3_dot;
      t58 = 0.5200e2 * t21;
      jac__1_2 = t14 * (t17 * (-0.5200e2 * t36 + t1 * (-t39 - 0.1578720e4 * theta2_dot) - 0.1052480e4 * t38 - 0.3596691746e5 * theta2_dot) + 0.5200e2 * t3 + t1 * (0.12480e3 * t6 + 0.884000e3 * theta4_dot) - 0.9533333333e2 * theta4_dot - 0.1326000e4 * t6) + t17 * (0.12480e3 * t56 + t1 * (-t58 - 0.799760e3 * theta3_dot) - 0.1673360e4 * t21 - 0.1095658600e6 * theta3_dot);
      t66 = 0.12480e3 * t36;
      t69 = -0.5200e2 * t38 + 0.526240e3 * theta2_dot;
      t71 = 0.6240e2 * theta2_dot;
      t72 = 0.789360e3 * t38;
      t75 = 0.5200e2 * t56;
      t84 = theta4_dot * t17;
      jac__1_3 = t15 * (t1 * t69 + t66 - t71 - t72) + t14 * (t75 + t1 * (t22 + 0.884000e3 * theta3_dot) - 0.9533333333e2 * theta3_dot - 0.1326000e4 * t21) + t1 * (-0.1852240e4 * theta2_dot - 0.1852240e4 * t84) + t5 * (-0.94640e2 * theta2_dot - 0.94640e2 * t84);
      t98 = theta2_dot * t17;
      jac__2_1 = (0.1248e3 * t5 * t34 + 0.52e2 * t3 + 0.4333333333e2 * theta4_dot + 0.52624e3 * t6 + 0.78936e3 * t34 + 0.52e2 * t2 * t98 + 0.1248e3 * t38 * t1 * t17 + 0.157872e4 * t1 * t98 + 0.105248e4 * t5 * t98 + 0.3596691746e5 * t98) * t14;
      jac__2_2 = theta4_dot * (0.52e2 * t5 * t1 + 0.78936e3 * t5 - 0.1248e3 * t2 + 0.624e2 - 0.52624e3 * t1);
      t116 = theta2_dot * t14;
      t118 = 0.12480e3 * theta3_dot;
      t122 = t39 + 0.7893600000e3 * theta2_dot;
      t124 = 0.526240e3 * theta3_dot;
      t127 = 0.4333333333e2 * theta2_dot;
      t131 = 0.6240e2 * theta3_dot;
      t132 = 0.789360e3 * t21;
      jac__2_3 = t2 * (0.5200000000e2 * t116 - t118) + t1 * (t14 * t122 - t124 + t58) + t14 * (t127 + 0.526240e3 * t38) + t131 + t132;
      jac__3_1 = t15 * (-t1 * t69 - t66 + t71 + t72) + t14 * (-t75 + t1 * (-0.7893600000e3 * theta3_dot - t22) - 0.5262400000e3 * t21 - 0.4333333333e2 * theta3_dot) + 0.1852240e4 * t1 * theta2_dot + 0.94640e2 * t38;
      jac__3_2 = t2 * (-0.5200e2 * t116 + t118) + t1 * (-t14 * t122 + t124 - t58) + t14 * (-0.5262400000e3 * t38 - t127) - t132 - t131;
      jac__3_3 = -0.2000e4;


      % store on output
      jac__gforce_v = zeros(3,3);
      jac__gforce_v(1,1) = jac__1_1;
      jac__gforce_v(1,2) = jac__1_2;
      jac__gforce_v(1,3) = jac__1_3;
      jac__gforce_v(2,1) = jac__2_1;
      jac__gforce_v(2,2) = jac__2_2;
      jac__gforce_v(2,3) = jac__2_3;
      jac__gforce_v(3,1) = jac__3_1;
      jac__gforce_v(3,2) = jac__3_2;
      jac__gforce_v(3,3) = jac__3_3;
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % d Phi(t,q) / dt     = Phi_q(t,q) v
    % d^2 Phi(t,q) / dt^2 = Phi_q(t,q) v' - b(t,q,v)
    function jac__b = b( self, t, pos, vel )
      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      
      theta2__dot = vel(1);
      theta3__dot = vel(2);
      theta4__dot = vel(3);
      
      % evaluate function
      jac__1_1 = theta2__dot - 0;
      jac__2_1 = theta3__dot - 0;

      % store on output
      jac__b = zeros(2,1);
      jac__b(1,1) = jac__1_1;
      jac__b(2,1) = jac__2_1;
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial b( t, q, v ) / \partial q
    function jac__b_q = b_q( t, pos, vel )
      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      
      theta2__dot = vel(1);
      theta3__dot = vel(2);
      theta4__dot = vel(3);
      % evaluate function
      
      % store on output
      jac__b_q = zeros(2,3);
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial b( t, q, v ) / \partial q
    function jac__b_v = b_v( t, pos, vel )
      % extract states
      theta2 = pos(1);
      theta3 = pos(2);
      theta4 = pos(3);
      
      theta2__dot = vel(1);
      theta3__dot = vel(2);
      theta4__dot = vel(3);
      
      % evaluate function
      jac__1_1 = 1;
      jac__2_2 = 1;
      
      % store on output
      jac__b_v = zeros(2,3);
      jac__b_v(1,1) = jac__1_1;
      jac__b_v(2,2) = jac__2_2;
      
    end 
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  end

  methods
    function self = PegasusImplicitDAE( )
      self@DAE3baseClassImplicit('PegasusImplicitDAE', 3, 2);
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function delete( self )
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function [n,m] = getDim( self )
      n = 3; % number of position/velocity coordinates
      m = 2;  % number of constraints
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function res = f( self, t, x )
      n  = self.npv;
      m  = self.nc;
      q  = x(1:n);
      v  = x(n+1:2*n);
      M  = self.M( t, q );
      Fq = self.Phi_q( t, q );
      f  = self.gforce( t, q, v, l );
      b  = self.b( t, q, v );

      MM    = [ M, Fq.'; Fq, zeros(m,m) ];
      vl    = MM \ [ f; b];
      v_dot = vl(1:n);

      res   = [ v; v_dot ];
    end
    
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    %
    %  return jacobian Df( t, x ) / Dx
    %
    function res = DfDx( self, t, x )
 
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    %
    %  return jacobian Df( t, x ) / Dt
    %
    function res = DfDt( self, t, x )
     
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    %
    %  return exact solution x such that x(t0) = x0
    %  if exact solution is not available define the function that return NaN
    %
    function res = exact( self, t0, x0, t )
      res = NaN;
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  end
end
