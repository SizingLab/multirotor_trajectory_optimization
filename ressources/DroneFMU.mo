within ;
package DroneFMU

  model DroneOptim_Torque
    Components.DroneMassPropeller_TorqueInput droneMassPropeller
      annotation (Placement(transformation(extent={{-10,-6},{10,14}})));

    Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(                         table=[0,
          n_hover; 1/17*Time,n1*n_hover; 2/17*Time,n2*n_hover; 3/17*Time,n3*
          n_hover; 4/17*Time,n4*n_hover; 5/17*Time,n5*n_hover; 6/17*Time,n6*
          n_hover; 7/17*Time,n7*n_hover; 8/17*Time,n8*n_hover; 9/17*Time,n9*
          n_hover; 10/17*Time,n10*n_hover; 11/17*Time,n11*n_hover; 12/17*Time,n12*
          n_hover; 13/17*Time,n13*n_hover; 14/17*Time,n14*n_hover; 15/17*Time,n15*
          n_hover; 16/17*Time,n16*n_hover; Time,n_hover], smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments)
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    parameter Real n_hover=droneMassPropeller.T_hover "Hover motor torque";
    parameter Real n1=1 "Speed coefficient";
    parameter Real n2=1 "Speed coefficient";
    parameter Real n3=1 "Speed coefficient";
    parameter Real n4=1 "Speed coefficient";
    parameter Real n5=1 "Speed coefficient";
    parameter Real n6=1 "Speed coefficient";
    parameter Real n7=1 "Speed coefficient";
    parameter Real n8=1 "Speed coefficient";
    parameter Real n9=1 "Speed coefficient";
    parameter Real n10=1 "Speed coefficient";
    parameter Real n11=1 "Speed coefficient";
    parameter Real n12=1 "Speed coefficient";
    parameter Real n13=1 "Speed coefficient";
    parameter Real n14=1 "Speed coefficient";
    parameter Real n15=1 "Speed coefficient";
    parameter Real n16=1 "Speed coefficient";
    parameter Real Time=5 "Travel Time";

    Modelica.Blocks.Continuous.Integrator NRJ
      annotation (Placement(transformation(extent={{60,-30},{80,-10}})));
  equation
    connect(NRJ.u, droneMassPropeller.Power) annotation (Line(points={{58,-20},{22,
            -20},{22,-4},{10,-4}},     color={0,0,127}));
    connect(combiTimeTable.y[1], droneMassPropeller.T) annotation (Line(points={{-59,
            0},{-36,0},{-36,0},{-10,0}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end DroneOptim_Torque;

  model DroneOptim_Speed

    Components.DroneMassPropeller_SpeedInput droneMassPropeller
      annotation (Placement(transformation(extent={{-12,-6},{8,14}})));
    Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(                         table=[0,
          n_hover; 1/17*Time,n1*n_hover; 2/17*Time,n2*n_hover; 3/17*Time,n3*
          n_hover; 4/17*Time,n4*n_hover; 5/17*Time,n5*n_hover; 6/17*Time,n6*
          n_hover; 7/17*Time,n7*n_hover; 8/17*Time,n8*n_hover; 9/17*Time,n9*
          n_hover; 10/17*Time,n10*n_hover; 11/17*Time,n11*n_hover; 12/17*Time,n12*
          n_hover; 13/17*Time,n13*n_hover; 14/17*Time,n14*n_hover; 15/17*Time,n15*
          n_hover; 16/17*Time,n16*n_hover; Time,n_hover], smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments)
      annotation (Placement(transformation(extent={{-52,8},{-32,28}})));
    parameter Real n_hover=droneMassPropeller.n_hover "Hover motor torque";
    parameter Real n1=1.15 "Speed coefficient";
    parameter Real n2=1.09 "Speed coefficient";
    parameter Real n3=1.09 "Speed coefficient";
    parameter Real n4=1.076 "Speed coefficient";
    parameter Real n5=1.063 "Speed coefficient";
    parameter Real n6=1.044 "Speed coefficient";
    parameter Real n7=1.031 "Speed coefficient";
    parameter Real n8=1.016 "Speed coefficient";
    parameter Real n9=0.999 "Speed coefficient";
    parameter Real n10=0.982 "Speed coefficient";
    parameter Real n11=0.966 "Speed coefficient";
    parameter Real n12=0.947 "Speed coefficient";
    parameter Real n13=0.933 "Speed coefficient";
    parameter Real n14=0.907 "Speed coefficient";
    parameter Real n15=0.905 "Speed coefficient";
    parameter Real n16=0.823 "Speed coefficient";
    parameter Real Time=5 "Travel Time";

    Modelica.Blocks.Continuous.Integrator NRJ
      annotation (Placement(transformation(extent={{28,-30},{48,-10}})));
  equation
    connect(NRJ.u, droneMassPropeller.Power) annotation (Line(points={{26,-20},
            {22,-20},{22,-4},{8,-4}},  color={0,0,127}));
    connect(droneMassPropeller.n, combiTimeTable.y[1])
      annotation (Line(points={{-12,0},{-20,0},{-20,18},{-31,18}},
                                                 color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
  end DroneOptim_Speed;

  model DroneOptim_SpeedSinus

    Components.DroneMassPropeller_SpeedInput droneMassPropeller
      annotation (Placement(transformation(extent={{-10,-6},{10,14}})));
    Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(                         table=[0,
          n_hover; 1/17*Time,n1*n_hover; 2/17*Time,n2*n_hover; 3/17*Time,n3*
          n_hover; 4/17*Time,n4*n_hover; 5/17*Time,n5*n_hover; 6/17*Time,n6*
          n_hover; 7/17*Time,n7*n_hover; 8/17*Time,n8*n_hover; 9/17*Time,n9*
          n_hover; 10/17*Time,n10*n_hover; 11/17*Time,n11*n_hover; 12/17*Time,n12*
          n_hover; 13/17*Time,n13*n_hover; 14/17*Time,n14*n_hover; 15/17*Time,n15*
          n_hover; 16/17*Time,n16*n_hover; Time,n_hover], smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments)
      annotation (Placement(transformation(extent={{-52,8},{-32,28}})));
    parameter Real n_hover=droneMassPropeller.n_hover "Hover motor torque";
    parameter Real n1=1.15 "Speed coefficient";
    parameter Real n2=1.09 "Speed coefficient";
    parameter Real n3=1.09 "Speed coefficient";
    parameter Real n4=1.076 "Speed coefficient";
    parameter Real n5=1.063 "Speed coefficient";
    parameter Real n6=1.044 "Speed coefficient";
    parameter Real n7=1.031 "Speed coefficient";
    parameter Real n8=1.016 "Speed coefficient";
    parameter Real n9=0.999 "Speed coefficient";
    parameter Real n10=0.982 "Speed coefficient";
    parameter Real n11=0.966 "Speed coefficient";
    parameter Real n12=0.947 "Speed coefficient";
    parameter Real n13=0.933 "Speed coefficient";
    parameter Real n14=0.907 "Speed coefficient";
    parameter Real n15=0.905 "Speed coefficient";
    parameter Real n16=0.823 "Speed coefficient";
    parameter Real Time=5 "Travel Time";

    Modelica.Blocks.Continuous.Integrator NRJ
      annotation (Placement(transformation(extent={{28,-30},{48,-10}})));
  equation
    connect(NRJ.u, droneMassPropeller.Power) annotation (Line(points={{26,-20},
            {22,-20},{22,-4},{10,-4}}, color={0,0,127}));
    connect(droneMassPropeller.n, combiTimeTable.y[1])
      annotation (Line(points={{-10,0},{-20,0},{-20,18},{-31,18}},
                                                 color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
  end DroneOptim_SpeedSinus;

  model TestProfilOptim
    Components.DroneMassPropeller_TorqueInput droneMassPropeller_TorqueInput
      annotation (Placement(transformation(extent={{30,-6},{50,14}})));
    Components.ProfilTable2 profilTable2_1(T=3)
      annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
    Modelica.Blocks.Math.Feedback feedback
      annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
    Modelica.Blocks.Math.Gain gain(k=100)
      annotation (Placement(transformation(extent={{-2,-6},{10,6}})));
    Modelica.Blocks.Continuous.SecondOrder secondOrder(D=1, w=2*3.14*20)
      annotation (Placement(transformation(extent={{-62,-10},{-42,10}})));
    Modelica.Blocks.Continuous.Integrator NRJ
      annotation (Placement(transformation(extent={{74,-60},{94,-40}})));
  equation
    connect(gain.u, feedback.y)
      annotation (Line(points={{-3.2,0},{-15,0}}, color={0,0,127}));
    connect(droneMassPropeller_TorqueInput.Acceleration, feedback.u2)
      annotation (Line(points={{50,8},{84,8},{84,-20},{-24,-20},{-24,-8}},
          color={0,0,127}));
    connect(secondOrder.y, feedback.u1) annotation (Line(points={{-41,0},{-36,0},
            {-36,0},{-32,0}}, color={0,0,127}));
    connect(secondOrder.u, profilTable2_1.zpp) annotation (Line(points={{-64,0},
            {-71,0},{-71,6},{-77,6}}, color={0,0,127}));
    connect(gain.y, droneMassPropeller_TorqueInput.T)
      annotation (Line(points={{10.6,0},{30,0}}, color={0,0,127}));
    connect(NRJ.u, droneMassPropeller_TorqueInput.Power) annotation (Line(
          points={{72,-50},{62,-50},{62,-4},{50,-4}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end TestProfilOptim;

  model TestProfilOptim_J
    Components.DroneMassPropeller_TorqueInput_J
      droneMassPropeller_TorqueInput_J
      annotation (Placement(transformation(extent={{52,-6},{72,14}})));
    Components.ProfilTable2 profilTable2_1(
      H=10,
      T=2.5457,
      Alpha=0.05)
      annotation (Placement(transformation(extent={{-152,-8},{-132,12}})));
    Modelica.Blocks.Math.Feedback feedback_acc
      annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
    Modelica.Blocks.Math.Gain gain_a(k=100)
      annotation (Placement(transformation(extent={{-52,-6},{-40,6}})));
    Modelica.Blocks.Continuous.SecondOrder secondOrder(D=1, w=2*3.14*20)
      annotation (Placement(transformation(extent={{-108,-10},{-88,10}})));
    Modelica.Blocks.Continuous.Integrator NRJ
      annotation (Placement(transformation(extent={{108,-30},{128,-10}})));
    Modelica.Blocks.Math.Gain gain_a1(k=1000) annotation (Placement(
          transformation(
          extent={{-6,-6},{6,6}},
          rotation=180,
          origin={0,40})));
    Modelica.Blocks.Math.Feedback add_saturation1 annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=180,
          origin={82,40})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=
          droneMassPropeller_TorqueInput_J.n_hover/10) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={112,40})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1e6, uMin=0) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={36,40})));
    Modelica.Blocks.Math.Add add
      annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
  equation
    connect(gain_a.u, feedback_acc.y)
      annotation (Line(points={{-53.2,0},{-59,0}}, color={0,0,127}));
    connect(droneMassPropeller_TorqueInput_J.Acceleration, feedback_acc.u2)
      annotation (Line(points={{72,7.4},{90,7.4},{90,-22},{-68,-22},{-68,-8}},
          color={0,0,127}));
    connect(secondOrder.y, feedback_acc.u1)
      annotation (Line(points={{-87,0},{-76,0}}, color={0,0,127}));
    connect(secondOrder.u, profilTable2_1.zpp) annotation (Line(points={{-110,0},
            {-119,0},{-119,8},{-131,8}},
                                      color={0,0,127}));
    connect(NRJ.u, droneMassPropeller_TorqueInput_J.Power) annotation (Line(
          points={{106,-20},{96,-20},{96,-4.6},{72,-4.6}},
                                                      color={0,0,127}));
    connect(droneMassPropeller_TorqueInput_J.n_mot, add_saturation1.u2)
      annotation (Line(points={{72,11.4},{80,11.4},{80,12},{82,12},{82,32}},
          color={0,0,127}));
    connect(realExpression.y, add_saturation1.u1)
      annotation (Line(points={{101,40},{90,40}}, color={0,0,127}));
    connect(limiter.u, add_saturation1.y)
      annotation (Line(points={{48,40},{73,40}}, color={0,0,127}));
    connect(limiter.y, gain_a1.u)
      annotation (Line(points={{25,40},{7.2,40}}, color={0,0,127}));
    connect(add.u2, gain_a.y)
      annotation (Line(points={{-8,0},{-39.4,0}}, color={0,0,127}));
    connect(add.u1, gain_a1.y) annotation (Line(points={{-8,12},{-22,12},{-22,
            40},{-6.6,40}}, color={0,0,127}));
    connect(add.y, droneMassPropeller_TorqueInput_J.T)
      annotation (Line(points={{15,6},{34,6},{34,0},{52,0}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
              -100},{140,100}})),                                  Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{140,
              100}})),
      experiment(StopTime=15.8, __Dymola_Algorithm="Dassl"));
  end TestProfilOptim_J;

  package Components
    model MultiRotor1D
      parameter Modelica.SIunits.Mass M = 1.35 "Multi-Rotor mass";
      parameter Modelica.SIunits.TranslationalSpringConstant K=10/2e-2
        "Spring constant of the landing gear";
      parameter Real Eps = 2/100 "Damping coefficient";
      parameter Modelica.SIunits.TranslationalDampingConstant d=2*Eps/sqrt(K/M)*K
        "Damping constant of the landing gear";

      Modelica.Mechanics.Translational.Components.Mass mass(m=M, a(start=0))
                                                            annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,-16})));
      DragForce dragForce(S=S, Cd=Cd) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,38})));
      Modelica.Blocks.Sources.RealExpression ZeroVerticalWindSpeed
        annotation (Placement(transformation(extent={{-30,54},{-10,74}})));
      Modelica.Mechanics.Translational.Sources.ConstantForce constantForce(
          f_constant=-M*9.81)
                             annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-34,30})));
      Modelica.Mechanics.Translational.Components.ElastoGap elastoGap(     d=d, c=K)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-34,-46})));
      Modelica.Mechanics.Translational.Components.Fixed fixed
        annotation (Placement(transformation(extent={{-44,-84},{-24,-64}})));

      Modelica.Mechanics.Translational.Interfaces.Flange_b PropellerThrust
        "Right flange of translational component"
        annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
      parameter Modelica.SIunits.Position Z0=-M*9.81/K
        "Initial vertical position";
      parameter Modelica.SIunits.Area S=0.1 "Projected surface";
      parameter Real Cd=1 "Drag coefficient";
    equation

      connect(ZeroVerticalWindSpeed.y, dragForce.v_wind)
        annotation (Line(points={{-9,64},{0,64},{0,48.6}}, color={0,0,127}));
      connect(fixed.flange, elastoGap.flange_a)
        annotation (Line(points={{-34,-74},{-34,-56}},
                                                   color={0,127,0}));
      connect(mass.flange_b, dragForce.flange)
        annotation (Line(points={{0,-6},{0,28}}, color={0,127,0}));
      connect(mass.flange_b, constantForce.flange) annotation (Line(points={{0,
              -6},{-18,-6},{-18,20},{-34,20}}, color={0,127,0}));
      connect(mass.flange_a, elastoGap.flange_b) annotation (Line(points={{0,
              -26},{-18,-26},{-18,-36},{-34,-36}}, color={0,127,0}));
      connect(mass.flange_a, PropellerThrust)
        annotation (Line(points={{0,-26},{0,-100}}, color={0,127,0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false), graphics={  Rectangle(extent = {{-64, 2}, {64, 0}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0},
                fillPattern =                                                                                                                                               FillPattern.Solid), Line(points = {{-30, -4}, {-60, -40}, {-68, -40}}, color = {0, 0, 0}), Ellipse(extent = {{-38, 8}, {36, -8}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{30, -4}, {60, -40}, {68, -40}}, color = {0, 0, 0}), Ellipse(extent = {{-84, 6}, {-64, 2}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Ellipse(extent = {{-64, 6}, {-44, 2}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Ellipse(extent = {{0, 6}, {20, 2}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Ellipse(extent = {{-20, 6}, {0, 2}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Ellipse(extent = {{64, 6}, {84, 2}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Ellipse(extent = {{44, 6}, {64, 2}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {28, 108, 200})}),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end MultiRotor1D;

    model MotoPropeller "Inertia Motor + Speed control + Propeller"
      Propeller propeller(
        d=d,
        Ct=Ct,
        Cp=Cp,
        Rho=Rho)
        annotation (Placement(transformation(extent={{64,-14},{88,14}})));
      Modelica.Mechanics.Translational.Interfaces.Flange_b flange_thrust annotation (
        Placement(transformation(extent = {{90, -10}, {110, 10}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertiaMotorPropeller(J = J, w(fixed=
              true, start=w_init))                                                  annotation (
        Placement(transformation(extent = {{24, -10}, {44, 10}})));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {54, -24})));
      Modelica.Blocks.Math.Feedback feedback annotation (
        Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
      Modelica.Blocks.Interfaces.RealInput Wref annotation (
        Placement(transformation(extent = {{-140, -20}, {-100, 20}}), iconTransformation(extent = {{-140, -20}, {-100, 20}})));
      Modelica.Blocks.Math.Gain gain(k = 100) annotation (
        Placement(transformation(extent = {{-54, -10}, {-34, 10}})));
      Modelica.Blocks.Nonlinear.Limiter Torque_limiter(uMax = Tmax, uMin=0)
                                                                    annotation (
        Placement(transformation(extent = {{-28, -10}, {-8, 10}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation (
        Placement(transformation(extent = {{0, -10}, {20, 10}})));
      parameter Modelica.SIunits.Length d = 10*2.5e-2 "Propeller diameter";
      parameter Real Ct = 0.1125 "Thrust coefficient";
      parameter Real Cp = 0.0254 "Torque coefficient";
      parameter Modelica.SIunits.Density Rho = 1.28 "Air density";
      parameter Modelica.SIunits.Inertia J = 1e-4 "Moment of inertia";
      parameter Modelica.SIunits.Torque Tmax = 0.1 "Max motor torque";
      parameter Modelica.SIunits.AngularVelocity Wmax = 1500 "Max motor speed";
      Modelica.Blocks.Nonlinear.Limiter Speed_limiter(uMax = Wmax) annotation (
        Placement(transformation(extent = {{-112, -10}, {-92, 10}})));
      parameter Modelica.SIunits.AngularVelocity w_init=500
        "Absolute angular velocity of component (= der(phi))";
    equation
      connect(propeller.flange_thrust, flange_thrust) annotation (
        Line(points = {{88, 1.77636e-15}, {94, 1.77636e-15}, {94, 0}, {100, 0}}, color = {0, 127, 0}));
      connect(inertiaMotorPropeller.flange_b, propeller.flange_motor) annotation (
        Line(points = {{44, 0}, {50, 0}, {50, 1.77636e-15}, {64, 1.77636e-15}}, color = {0, 0, 0}));
      connect(speedSensor.flange, propeller.flange_motor) annotation (
        Line(points = {{54, -14}, {54, 1.77636e-15}, {64, 1.77636e-15}}, color = {0, 0, 0}));
      connect(gain.u, feedback.y) annotation (
        Line(points = {{-56, 0}, {-61, 0}}, color = {0, 0, 127}));
      connect(gain.y, Torque_limiter.u) annotation (
        Line(points = {{-33, 0}, {-30, 0}}, color = {0, 0, 127}));
      connect(speedSensor.flange, inertiaMotorPropeller.flange_b) annotation (
        Line(points = {{54, -14}, {54, 0}, {44, 0}}, color = {0, 0, 0}));
      connect(inertiaMotorPropeller.flange_a, torque.flange) annotation (
        Line(points = {{24, 0}, {20, 0}}, color = {0, 0, 0}));
      connect(Torque_limiter.y, torque.tau) annotation (
        Line(points = {{-7, 0}, {-2, 0}}, color = {0, 0, 127}));
      connect(speedSensor.w, feedback.u2) annotation (
        Line(points = {{54, -35}, {54, -40}, {-70, -40}, {-70, -8}}, color = {0, 0, 127}));
      connect(Speed_limiter.y, feedback.u1) annotation (
        Line(points = {{-91, 0}, {-78, 0}}, color = {0, 0, 127}));
      connect(Speed_limiter.u, Wref) annotation (
        Line(points = {{-114, 0}, {-120, 0}}, color = {0, 0, 127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-140, -100}, {100, 100}}), graphics={  Line(points = {{90, 0}, {68, 0}}, color = {0, 140, 72}), Line(points = {{0, 0}, {22, 0}}, color = {0, 0, 0}), Rectangle(extent = {{22, 8}, {62, -8}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder), Ellipse(extent = {{54, 8}, {68, -8}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder), Ellipse(extent = {{52, 100}, {72, 0}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder), Ellipse(extent = {{52, 0}, {72, -100}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder), Rectangle(extent = {{-40, 40}, {26, -40}}, lineColor = {0, 0, 0},
                fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {0, 128, 255}), Rectangle(extent = {{-40, 40}, {-60, -40}}, lineColor = {0, 0, 0},
                fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {128, 128, 128}), Rectangle(extent = {{26, 8}, {44, -8}}, lineColor = {0, 0, 0},
                fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, fillColor = {95, 95, 95}), Line(points = {{-100, 0}, {-60, 0}}, color = {28, 108, 200})}),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, -100}, {100, 100}})));
    end MotoPropeller;

    model DragForce
      "Quadratic dependency of force versus speed (aerodynamic drag force)"
      extends Modelica.Mechanics.Translational.Interfaces.PartialForce;
      parameter Modelica.SIunits.Area S = 0.1 "Projected surface";
      parameter Real Cd = 1 "Drag coefficient";
      parameter Modelica.SIunits.Density Rho = 1.28 "Air density";
      Modelica.SIunits.Velocity v "Velocity of flange with respect to support (= der(s))";
      Modelica.Blocks.Interfaces.RealInput v_wind annotation (
        Placement(transformation(extent = {{-126, -20}, {-86, 20}})));
    equation
      v = der(s);
      f = 1 / 2 * Rho * Cd * S * abs(v - v_wind) * (v - v_wind);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{-100, -100}, {-80, -98}, {-60, -92}, {-40, -82}, {-20, -68}, {0, -50}, {20, -28}, {40, -2}, {60, 28}, {80, 62}, {100, 100}}, color = {0, 0, 127}, smooth = Smooth.Bezier)}),
        Documentation(info = "<html>
<p>
Model of force, quadratic dependent on velocity of flange.<br>
Parameter ForceDirection chooses whether direction of force is the same in both directions of movement or not.
</p>
</html>"));
    end DragForce;

    model DroneMassPropeller_TorqueInput

        // The drone parameters
        parameter Real Np=4 "Propeller number";
        parameter Real M=1.3 "Drone mass";
        parameter Real beta=0.34 "Pitch angle coefficient";
        parameter Real Ct0=0.02791+0.11867*beta+0.27334*beta^2 - 0.28852*beta^3 "Static Thrust coefficient";
        parameter Real Cp0=0.01813-0.06218*beta+0.35712*beta^2 - 0.23774*beta^3 "Static Power coefficient";
        parameter Real Rho=1.18 "Air mass volumic";
        parameter Real d=10*0.0254 "Propeller diameter";
        parameter Real S=0.4*0.4 "Surface";
        parameter Real Cd=1 "Drag coefficient";
        parameter Real J_pro=1e-4 "Propeller inertia";
        parameter Real K_mot=(1./760.) * 60./2./3.14 "[V/(rad/s)] Motor torque coeff";
        parameter Real R_mot = 0.26 "[Ohm] Motor resistance";
        parameter Real g=9.81 "Gravity coeffcient";

        // The mission parameters
        parameter Real H=10 "Height";
        //
        parameter Real n_hover=sqrt(M*g/Rho/Ct0/d^4/Np) "Hover rotational speed";
        parameter Real T_hover=Rho*Cp0*n_hover^2*d^5/(2*3.14) "Hover motor torque";


        // Aero coefficient
        Real Ct "Thrust coefficient";
        Real Cp "Power coefficient";
        Real J "Advance ratio";

        // The states
        Real xp(start=0,fixed=true) "Speed";
        Real x(start=0,fixed=true) "Position";
        // The force and power
        Real F "Propeller force";
        Real P "Propeller power";
        Real n(start=n_hover,fixed=true) "Propeller speed (Hz)";
        Real w "Motor speed (rad/s)";
        Real Pmot "Motor losses";

        // Constraints variable

      Modelica.Blocks.Interfaces.RealInput T
        annotation (Placement(transformation(extent={{-120,-60},{-80,-20}})));
      Modelica.Blocks.Interfaces.RealOutput Speed
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.RealOutput Position
        annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
      Modelica.Blocks.Interfaces.RealOutput Power
        annotation (Placement(transformation(extent={{90,-90},{110,-70}})));
      Modelica.Blocks.Interfaces.RealOutput Acceleration
        annotation (Placement(transformation(extent={{90,30},{110,50}})));
    equation
        // Advance ratio
        J=xp/n/d;

        // C_t and C_d for APC props in dynamics
        Ct=0.02791-0.06543*J+0.11867*beta+0.27334*beta^2 - 0.28852*beta^3 + 0.02104
        *J^3 - 0.23504*J^2 + 0.18677*beta*J^2;
        Cp=0.01813-0.06218*beta+0.00343*J+0.35712*beta^2 - 0.23774*beta^3 + 0.07549
        *beta*J - 0.1235*J^2;

        // Mechanical equations
        der(x) = xp; // speed
        F=Rho*Ct*n^2*d^4*Np; // Propellers force
        M*der(xp) = F -1/2*Cd*S*xp*sign(xp) -M*g; // PFD
        P=Rho*Ct*n^3*d^5*Np; //Propellers power
        J_pro*der(w)=T-Rho*Cp*n^2*d^5/(2*3.14); // Motor torque
        w=n*2*3.14; // Speeds
        Pmot=R_mot*(T/K_mot)^2; // Motor losses

        //Output
        Acceleration=der(xp);
        Speed=xp;
        Position=x;
        Power=P+Np*Pmot;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-78,0},{80,-80}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0}),
            Ellipse(
              extent={{-100,40},{0,20}},
              lineColor={28,108,200},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={255,0,0}),
            Ellipse(
              extent={{0,40},{100,20}},
              lineColor={28,108,200},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={255,0,0}),
            Rectangle(
              extent={{-4,30},{4,0}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0}),
            Text(
              extent={{-120,12},{-84,-24}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0},
              textString="T")}),                                     Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end DroneMassPropeller_TorqueInput;

    model DroneMassPropeller_SpeedInput

        // The drone parameters
        parameter Real Np=4 "Propeller number";
        parameter Real M=1.3 "Drone mass";
        parameter Real beta=0.34 "Pitch angle coefficient";
        parameter Real Ct0=0.02791+0.11867*beta+0.27334*beta^2 - 0.28852*beta^3 "Static Thrust coefficient";
        parameter Real Cp0=0.01813-0.06218*beta+0.35712*beta^2 - 0.23774*beta^3 "Static Power coefficient";
        parameter Real Rho=1.18 "Air mass volumic";
        parameter Real d=10*0.0254 "Propeller diameter";
        parameter Real S=0.4*0.4 "Surface";
        parameter Real Cd=1 "Drag coefficient";
        parameter Real J_pro=1e-4 "Propeller inertia";
        parameter Real K_mot=(1./760.) * 60./2./3.14 "[V/(rad/s)] Motor torque coeff";
        parameter Real R_mot = 0.26 "[Ohm] Motor resistance";
        parameter Real g=9.81 "Gravity coeffcient";

        // The mission parameters
        parameter Real H=10 "Height";
        //
        parameter Real n_hover=sqrt(M*g/Rho/Ct0/d^4/Np) "Hover rotational speed";
        parameter Real T_hover=Rho*Cp0*n_hover^2*d^5/(2*3.14) "Hover motor torque";

        // Aero coefficient
        Real Ct "Thrust coefficient";
        Real Cp "Power coefficient";
        Real J "Advance ratio";
        // The states
        // The states
        Real xp(start=0,fixed=true) "Speed";
        Real x(start=0,fixed=true) "Position";
        // The force and power
        Real F "Propeller force";
        Real P "Propeller power";
        Real T "Propeller or motor torque";
        //Real n(start=n_hover,fixed=true) "Propeller speed (Hz)";
        Real w "Motor speed (rad/s)";
        Real Pmot "Motor losses";

        // Constraints variable

      Modelica.Blocks.Interfaces.RealInput n
        annotation (Placement(transformation(extent={{-120,-60},{-80,-20}})));
      Modelica.Blocks.Interfaces.RealOutput Speed
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.RealOutput Position
        annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
      Modelica.Blocks.Interfaces.RealOutput Power
        annotation (Placement(transformation(extent={{90,-90},{110,-70}})));
    equation
        // Advance ratio
        J=xp/n/d;

        // C_t and C_d for APC props in dynamics
        Ct=0.02791-0.06543*J+0.11867*beta+0.27334*beta^2 - 0.28852*beta^3 + 0.02104
        *J^3 - 0.23504*J^2 + 0.18677*beta*J^2;
        Cp=0.01813-0.06218*beta+0.00343*J+0.35712*beta^2 - 0.23774*beta^3 + 0.07549
        *beta*J - 0.1235*J^2;

        // Mechanical equations
        der(x) = xp; // speed
        F=Rho*Ct*n^2*d^4*Np; // Propellers force
        M*der(xp) = F -1/2*Cd*S*xp*sign(xp) -M*g; // PFD
        P=Rho*Ct*n^3*d^5*Np; //Propellers power
        0=T-Rho*Cp*n^2*d^5/(2*3.14); // Motor torque
        w=n*2*3.14; // Speeds
        Pmot=R_mot*(T/K_mot)^2; // Motor losses

        //Output
        Speed=xp;
        Position=x;
        Power=P+Np*Pmot;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-78,0},{80,-80}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0}),
            Ellipse(
              extent={{-100,40},{0,20}},
              lineColor={28,108,200},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={255,0,0}),
            Ellipse(
              extent={{0,40},{100,20}},
              lineColor={28,108,200},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={255,0,0}),
            Rectangle(
              extent={{-4,30},{4,0}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0}),
            Text(
              extent={{-120,12},{-84,-24}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0},
              textString="n"),
            Text(
              extent={{96,-44},{132,-80}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0},
              textString="P"),
            Text(
              extent={{96,-4},{132,-40}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0},
              textString="z"),
            Text(
              extent={{96,36},{132,0}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0},
              textString="v")}),                                     Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end DroneMassPropeller_SpeedInput;

    model ProfilTable

      parameter Real Z0=0 "Initial position";
      parameter Real H=10 "Elevation";
      parameter Real T=5 "Displacement time";
      parameter Real Vmax=3/2*H/T "Max speed";
      parameter Real Amax=4*Vmax/T "Max acceleration";
      Modelica.Blocks.Sources.TimeTable Acceleration(table=[0,Amax; T,-Amax; T,0])
        annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
      Modelica.Blocks.Continuous.Integrator Speed
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Modelica.Blocks.Continuous.Integrator Position(y_start=Z0)
        annotation (Placement(transformation(extent={{30,-10},{50,10}})));
      Modelica.Blocks.Interfaces.RealOutput z "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,-70},{120,-50}}),
            iconTransformation(extent={{100,-70},{120,-50}})));
      Modelica.Blocks.Interfaces.RealOutput zp "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}}),
            iconTransformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.RealOutput zpp "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
    equation
      connect(Speed.u, Acceleration.y)
        annotation (Line(points={{-12,0},{-29,0}}, color={0,0,127}));
      connect(Position.u, Speed.y)
        annotation (Line(points={{28,0},{11,0}}, color={0,0,127}));
      connect(Position.y, z) annotation (Line(points={{51,0},{76,0},{76,-60},{110,-60}},
            color={0,0,127}));
      connect(Speed.y, zp) annotation (Line(points={{11,0},{16,0},{16,30},{110,30},{
              110,0}}, color={0,0,127}));
      connect(Acceleration.y, zpp) annotation (Line(points={{-29,0},{-20,0},{-20,60},
              {110,60}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Line(points={{-84,60},{-60,60},{-60,80},{60,40},{60,60},{80,60}}, color=
                 {28,108,200}),
            Line(points={{-84,0},{-60,0},{-30,28},{28,28},{60,0},{80,0}}, color={28,
                  108,200}),
            Line(points={{-84,-80},{-60,-80},{-30,-72},{32,-40},{60,-34},{80,-34}},
                color={28,108,200}),
            Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
            Text(
              extent={{-22,-40},{22,-74}},
              lineColor={28,108,200},
              textString="z"),
            Text(
              extent={{-22,20},{22,-14}},
              lineColor={28,108,200},
              textString="v"),
            Text(
              extent={{-22,80},{22,46}},
              lineColor={28,108,200},
              textString="a")}),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
    end ProfilTable;

    model ProfilTable2

      parameter Real Z0=0 "Initial position";
      parameter Real H=10 "Elevation";
      parameter Real T=5 "Displacement time";
      parameter Real Alpha=0.1
                              "Inertia acceleration time ratio";
      parameter Real Vmax=Amax/4*T "Max speed";
      parameter Real Amax=6*H/T^2/(1-Alpha) "Max acceleration";
      Modelica.Blocks.Sources.TimeTable Acceleration(table=[0,0; Alpha*T,Amax; T/
            2,0; (1 - Alpha)*T,-Amax; T,0; 2*T,0])
        annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
      Modelica.Blocks.Continuous.Integrator Speed
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Modelica.Blocks.Continuous.Integrator Position(y_start=Z0)
        annotation (Placement(transformation(extent={{30,-10},{50,10}})));
      Modelica.Blocks.Interfaces.RealOutput z "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,-70},{120,-50}}),
            iconTransformation(extent={{100,-70},{120,-50}})));
      Modelica.Blocks.Interfaces.RealOutput zp "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}}),
            iconTransformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.RealOutput zpp "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
    equation
      connect(Speed.u, Acceleration.y)
        annotation (Line(points={{-12,0},{-29,0}}, color={0,0,127}));
      connect(Position.u, Speed.y)
        annotation (Line(points={{28,0},{11,0}}, color={0,0,127}));
      connect(Position.y, z) annotation (Line(points={{51,0},{76,0},{76,-60},{110,-60}},
            color={0,0,127}));
      connect(Speed.y, zp) annotation (Line(points={{11,0},{16,0},{16,30},{110,30},{
              110,0}}, color={0,0,127}));
      connect(Acceleration.y, zpp) annotation (Line(points={{-29,0},{-20,0},{-20,60},
              {110,60}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Line(points={{-84,60},{-60,60},{-60,80},{60,40},{60,60},{80,60}}, color=
                 {28,108,200}),
            Line(points={{-84,0},{-60,0},{-30,28},{28,28},{60,0},{80,0}}, color={28,
                  108,200}),
            Line(points={{-84,-80},{-60,-80},{-30,-72},{32,-40},{60,-34},{80,-34}},
                color={28,108,200}),
            Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
            Text(
              extent={{-22,-40},{22,-74}},
              lineColor={28,108,200},
              textString="z"),
            Text(
              extent={{-22,20},{22,-14}},
              lineColor={28,108,200},
              textString="v"),
            Text(
              extent={{-22,80},{22,46}},
              lineColor={28,108,200},
              textString="a")}),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
    end ProfilTable2;

    model DroneMassPropeller_TorqueInput_J

        // The drone parameters
        parameter Real Np=4 "Propeller number";
        parameter Real M=1.3 "Drone mass";
        parameter Real beta=0.3 "Pitch angle coefficient";
        parameter Real Rho=1.18 "Air mass volumic";
        parameter Real d=10*0.0254 "Propeller diameter";
        parameter Real S=0.4*0.4 "Surface";
        parameter Real Cd=1 "Drag coefficient";
        parameter Real J_pro=1e-4 "Propeller inertia";
        parameter Real K_mot=(1./760.) * 60./2./3.14 "[V/(rad/s)] Motor torque coeff";
        parameter Real R_mot = 0.26 "[Ohm] Motor resistance";
        parameter Real g=9.81 "Gravity coeffcient";

        // The mission parameters
        parameter Real H=10 "Height";
        //
        parameter Real Ct0 = 0.02791+0.11867*beta + 0.27334*beta^2 - 0.28852*beta^3 "Static thrust coef";
        parameter Real Cp0 = 0.01813-0.06218*beta + 0.35712*beta^2 - 0.23774*beta^3 "Satic power coef";
        //
        parameter Real n_hover=sqrt(M*g/Rho/Ct0/d^4/Np) "Hover rotational speed";
        parameter Real T_hover=Rho*Cp0*n_hover^2*d^5/(2*3.14) "Hover motor torque";

        // Aero coefficient
        Real Ct "Thrust coefficient";
        Real Cp "Power coefficient";
        Real J "Advance ratio";
        // The states
        Real xp(start=0,fixed=true) "Speed";
        Real x(start=0,fixed=true) "Position";
        // The force and power
        Real F "Propeller force";
        Real P "Propeller power";
        Real n(start=n_hover,fixed=true) "Propeller speed (Hz)";
        Real w "Motor speed (rad/s)";
        Real Pmot "Motor losses";

        // Constraints variable

      Modelica.Blocks.Interfaces.RealInput T
        annotation (Placement(transformation(extent={{-120,-60},{-80,-20}})));
      Modelica.Blocks.Interfaces.RealOutput Speed
        annotation (Placement(transformation(extent={{90,-16},{110,4}})));
      Modelica.Blocks.Interfaces.RealOutput Position
        annotation (Placement(transformation(extent={{90,-56},{110,-36}})));
      Modelica.Blocks.Interfaces.RealOutput Power
        annotation (Placement(transformation(extent={{90,-96},{110,-76}})));
      Modelica.Blocks.Interfaces.RealOutput Acceleration
        annotation (Placement(transformation(extent={{90,24},{110,44}})));
      Modelica.Blocks.Interfaces.RealOutput n_mot
        annotation (Placement(transformation(extent={{90,64},{110,84}})));
    equation
        // Advance ratio
        J=xp/n/d;

        // C_t and C_d for APC props in dynamics
        Ct=0.02791-0.06543*J+0.11867*beta+0.27334*beta^2 - 0.28852*beta^3 + 0.02104
        *J^3 - 0.23504*J^2 + 0.18677*beta*J^2;
        Cp=0.01813-0.06218*beta+0.00343*J+0.35712*beta^2 - 0.23774*beta^3 + 0.07549
        *beta*J - 0.1235*J^2;

        // Mechanical equations
        der(x) = xp; // speed
        F=Rho*Ct*n^2*d^4*Np; // Propellers force
        M*der(xp) = F -1/2*Cd*S*xp*sign(xp) -M*g; // PFD

        P=Rho*Ct*n^3*d^5*Np; //Propellers power
        J_pro*der(w)=T-Rho*Cp*n^2*d^5/(2*3.14); // Motor torque
        w=n*2*3.14; // Speeds
        Pmot=R_mot*(T/K_mot)^2; // Motor losses

        //Output
        Acceleration=der(xp);
        Speed=xp;
        Position=x;
        Power=P+Np*Pmot;
        n_mot=n;

        // Arret si vitesse devient trop petite
        assert(n  > 0.05*n_hover, "Propeller rotational speed too low");

      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-78,0},{80,-80}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0}),
            Ellipse(
              extent={{-100,40},{0,20}},
              lineColor={28,108,200},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={255,0,0}),
            Ellipse(
              extent={{0,40},{100,20}},
              lineColor={28,108,200},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={255,0,0}),
            Rectangle(
              extent={{-4,30},{4,0}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0}),
            Text(
              extent={{-120,12},{-84,-24}},
              lineColor={28,108,200},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={255,0,0},
              textString="T")}),                                     Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end DroneMassPropeller_TorqueInput_J;

    model Propeller "Propeller force generator"
      parameter Modelica.SIunits.Length d = 0.1 "Propeller diameter";
      parameter Real Ct = 0.1125 "Thrust coefficient";
      parameter Real Cp = 0.0254 "Power or torque coefficient";
      parameter Modelica.SIunits.Density Rho = 1.28 "Air density";
      Modelica.SIunits.AngularVelocity Omega "Angular velocity of the propeller";
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_motor annotation (
        Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
      Modelica.Mechanics.Translational.Interfaces.Flange_b flange_thrust annotation (
        Placement(transformation(extent = {{90, -10}, {110, 10}})));
    equation
      Omega = der(flange_motor.phi);
      // Thrust
      flange_thrust.f = -Ct * Rho * (Omega/2/3.14) ^ 2 * d ^ 4;
      // Torque
      flange_motor.tau = Cp * Rho * (Omega)^2 * (1/2/3.14)^3 * d ^ 5;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false), graphics={  Line(points = {{96, 0}, {6, 0}}, color = {0, 140, 72}), Line(points = {{-96, 0}, {-40, 0}}, color = {0, 0, 0}), Rectangle(extent = {{-40, 8}, {0, -8}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder), Ellipse(extent = {{-8, 8}, {6, -8}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder), Ellipse(extent = {{-10, 100}, {10, 0}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder), Ellipse(extent = {{-10, 0}, {10, -100}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder)}),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end Propeller;

    model Propeller_J "Propeller force generator"
      parameter Modelica.SIunits.Length d = 0.1 "Propeller diameter";
      parameter Modelica.SIunits.Density Rho = 1.18 "Air density";
      parameter Real beta=0.3 "Pitch angle coefficient";

      // Aero coefficient
      Real Ct "Thrust coefficient";
      Real Cp "Power coefficient";
      Real J "Advance ratio";
      Real n "Rotational speed (Hz)";
      Real xp "Propeller speed";

      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_motor annotation (
        Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
      Modelica.Mechanics.Translational.Interfaces.Flange_b flange_thrust annotation (
        Placement(transformation(extent = {{90, -10}, {110, 10}})));
    equation
      // Propeller advance speed
      xp=der(flange_thrust.s);

      // Advance ratio
      J=xp/n/d;

      // C_t and C_d for APC props in dynamics
      Ct=0.02791-0.06543*J+0.11867*beta+0.27334*beta^2 - 0.28852*beta^3 + 0.02104*J^3 - 0.23504*J^2 + 0.18677*beta*J^2;
      Cp=0.01813-0.06218*beta+0.00343*J+0.35712*beta^2 - 0.23774*beta^3 + 0.07549*beta*J - 0.1235*J^2;

      n = der(flange_motor.phi)/2/3.14;

      // Thrust
      flange_thrust.f = -Ct * Rho * n ^ 2 * d ^ 4;
      // Torque
      flange_motor.tau = Cp * Rho * n ^ 2 * (1/2/3.14) * d ^ 5;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false), graphics={  Line(points = {{96, 0}, {6, 0}}, color = {0, 140, 72}), Line(points = {{-96, 0}, {-40, 0}}, color = {0, 0, 0}), Rectangle(extent = {{-40, 8}, {0, -8}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder), Ellipse(extent = {{-8, 8}, {6, -8}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder), Ellipse(extent = {{-10, 100}, {10, 0}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder), Ellipse(extent = {{-10, 0}, {10, -100}}, lineColor = {28, 108, 200}, fillColor = {28, 108, 200},
                fillPattern =                                                                                                                                                                                                        FillPattern.VerticalCylinder)}),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end Propeller_J;

    model HarmonicSource
      import    Modelica.Constants.pi;

      parameter Integer n_h=15 "Number of harmonic";
      parameter Modelica.SIunits.Time Time=5 "Simulation time";
      parameter Modelica.SIunits.Frequency f0=1/(2*Time) "Fondamental frequency";
      parameter Modelica.SIunits.Frequency f[n_h]=f0*(1:n_h) "Harmonic";
      parameter Real amplitude[n_h]=ones(n_h)./(1:n_h) "Amplitude of sine wave";
      Modelica.Blocks.Sources.Sine sine[n_h](
        amplitude=amplitude,                 freqHz=f, offset=0)
        annotation (Placement(transformation(extent={{-40,-70},{-20,-50}})));

      Modelica.Blocks.Math.Sum sum1(nin=n_h)
        annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
    equation
      connect(sum1.u, sine.y)
        annotation (Line(points={{-2,-60},{-19,-60}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end HarmonicSource;

    model VectorSine
      Modelica.Blocks.Sources.Sine sine
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end VectorSine;
  end Components;
  annotation (uses(Modelica(version="3.2.3")));
end DroneFMU;
