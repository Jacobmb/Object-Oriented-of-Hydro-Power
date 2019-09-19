within ;
package MyWork "Content of the tutorial FM3217"

  package Tutorial1 "Contents of tutorial 1"
    model SimplePendulum "Model of a simple pendulum"
      //Import of packages
      import SI = Modelica.SIunits;

      //declaration of parameters
      parameter SI.Length L(min=0)= 1 "length of pendulum";
      constant SI.Acceleration g=9.81 "gravitational force";
      Real theta(start=0.1, fixed=true) "Displacement angle of pendulum";
      Real thetadot " Angle velocity";

      //declaration of equations
    equation
      thetadot = der(theta);
      der(thetadot) = -g/L*sin(theta);
      annotation (experiment(StopTime=10));
    end SimplePendulum;

    model SimplePendulumSIunits "Model of a simple pendulum, SI units"
      //Import of packages
      import SI = Modelica.SIunits;

      //declaration of parameters
      parameter SI.Length L(min=0)= 1 "length of pendulum";
      constant SI.Acceleration g=9.81 "gravitational force";
      Real theta(start=0.1, fixed=true) "Displacement angle of pendulum";
      Real thetadot " Angle velocity";

      //declaration of equations
    equation
      thetadot = der(theta);
      der(thetadot) = -g/L*sin(theta);
      annotation (experiment(StopTime=10));
    end SimplePendulumSIunits;

    model SimplePendulumSIunitsUsingImports
      "Model of a simple pendulum, SI units"
      //Import of packages
      import SI = Modelica.SIunits;

      //declaration of parameters
      parameter SI.Length L(min=0)= 1 "length of pendulum";
      constant SI.Acceleration g=9.81 "gravitational force";
      Real theta(start=0.1, fixed=true) "Displacement angle of pendulum";
      Real thetadot " Angle velocity";

      //declaration of equations
    equation
      thetadot = der(theta);
      der(thetadot) = -g/L*sin(theta);
      annotation (experiment(StopTime=10));
    end SimplePendulumSIunitsUsingImports;
  end Tutorial1;

  package Tutorial2 "Graphical modelling, contents of tutorial 2"
    model DCMotor
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5)
        annotation (Placement(transformation(extent={{-46,60},{-26,80}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.05)
        annotation (Placement(transformation(extent={{-10,60},{10,80}})));
      Modelica.Electrical.Analog.Basic.EMF emf
        annotation (Placement(transformation(extent={{14,14},{34,34}})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-68,-24},{-48,-4}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-58,22})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
        annotation (Placement(transformation(extent={{40,14},{60,34}})));
      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-142,-20},{-102,20}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
                        "Flange of right shaft"
        annotation (Placement(transformation(extent={{98,-10},{118,10}})));
    equation
      connect(signalVoltage.p, resistor.p) annotation (Line(points={{-58,32},{
              -58,70},{-46,70}}, color={0,0,255}));
      connect(resistor.n, inductor.p)
        annotation (Line(points={{-26,70},{-10,70}}, color={0,0,255}));
      connect(inductor.n, emf.p)
        annotation (Line(points={{10,70},{24,70},{24,34}}, color={0,0,255}));
      connect(signalVoltage.n, emf.n) annotation (Line(points={{-58,12},{-58,2},
              {24,2},{24,14}}, color={0,0,255}));
      connect(emf.flange, inertia.flange_a)
        annotation (Line(points={{34,24},{40,24}}, color={0,0,0}));
      connect(signalVoltage.n, ground.p)
        annotation (Line(points={{-58,12},{-58,-4}}, color={0,0,255}));
      connect(signalVoltage.v, u) annotation (Line(points={{-70,22},{-96,22},{
              -96,0},{-122,0}}, color={0,0,127}));
      connect(inertia.flange_b, flange_b1) annotation (Line(points={{60,24},{84,
              24},{84,0},{108,0}}, color={0,0,0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={Bitmap(
                extent={{-104,-100},{102,100}}, fileName=
                  "modelica://MyWork/4055cd27ff372773233059c628df2583b (1).jpg")}),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=10));
    end DCMotor;

    model MotorDrive
      DCMotor dCMotor
        annotation (Placement(transformation(extent={{-8,6},{12,26}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
        annotation (Placement(transformation(extent={{62,6},{82,26}})));
      Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
        annotation (Placement(transformation(extent={{30,6},{50,26}})));
      Modelica.Blocks.Continuous.PID PID
        annotation (Placement(transformation(extent={{-42,6},{-22,26}})));
      Modelica.Blocks.Sources.Step step(height=0.5)
        annotation (Placement(transformation(extent={{-96,6},{-76,26}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
         Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=90,
            origin={86,-30})));
      Modelica.Blocks.Math.Feedback feedback
        annotation (Placement(transformation(extent={{-70,6},{-50,26}})));
    equation
      connect(inertia.flange_a, idealGear.flange_b)
        annotation (Line(points={{62,16},{50,16}}, color={0,0,0}));
      connect(inertia.flange_b, angleSensor.flange)
        annotation (Line(points={{82,16},{86,16},{86,-20}}, color={0,0,0}));
      connect(step.y, feedback.u1)
        annotation (Line(points={{-75,16},{-68,16}}, color={0,0,127}));
      connect(feedback.y, PID.u)
        annotation (Line(points={{-51,16},{-44,16}}, color={0,0,127}));
      connect(angleSensor.phi, feedback.u2) annotation (Line(points={{86,-41},{
              86,-50},{-60,-50},{-60,8}}, color={0,0,127}));
      connect(PID.y, dCMotor.u) annotation (Line(points={{-21,16},{-16,16},{-16,
              16},{-10.2,16}}, color={0,0,127}));
      connect(dCMotor.flange_b1, idealGear.flange_a)
        annotation (Line(points={{12.8,16},{30,16}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end MotorDrive;
  end Tutorial2;

  package Tutorial4
    model ElectricKettle
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=230^2/2000,
          useHeatPort=true) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=-90,
            origin={-14,6})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-90,-62},{-70,-42}})));
      Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(V=sqrt(2)*230,
          freqHz=50) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=-90,
            origin={-80,0})));
      Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
        annotation (Placement(transformation(extent={{-42,24},{-22,44}})));
      Modelica.Blocks.Math.Mean mean(f=50) annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=-90,
            origin={-51,7})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Water(C=1.7*4.18e3,
          T(start=283.15, fixed=true))
        annotation (Placement(transformation(extent={{6,16},{26,36}})));
      Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor temperatureSensor
        annotation (Placement(transformation(extent={{28,-4},{48,16}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor KettleWall(G=5)
        annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=90,
            origin={16,-30})));
      Modelica.Thermal.HeatTransfer.Celsius.FixedTemperature RoomTemperature(T=
            20)
        annotation (Placement(transformation(extent={{-22,-56},{-2,-36}})));
      Modelica.Blocks.Logical.OnOffController onOffController(bandwidth=3)
        annotation (Placement(transformation(extent={{64,4},{74,14}})));
      Modelica.Blocks.Sources.Constant const(k=95)
        annotation (Placement(transformation(extent={{28,38},{44,54}})));
      Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch1
        annotation (Placement(transformation(extent={{-70,28},{-58,40}})));
    equation
      connect(sineVoltage.n, resistor.n) annotation (Line(points={{-80,-10},{
              -80,-24},{-14,-24},{-14,-4}}, color={0,0,255}));
      connect(ground.p, resistor.n) annotation (Line(points={{-80,-42},{-80,-24},
              {-14,-24},{-14,-4}}, color={0,0,255}));
      connect(powerSensor.nc, resistor.p) annotation (Line(points={{-22,34},{
              -14,34},{-14,16}}, color={0,0,255}));
      connect(powerSensor.pv, resistor.p) annotation (Line(points={{-32,44},{-8,
              44},{-8,20},{-14,20},{-14,16}}, color={0,0,255}));
      connect(powerSensor.nv, resistor.n) annotation (Line(points={{-32,24},{
              -32,-10},{-14,-10},{-14,-4}}, color={0,0,255}));
      connect(powerSensor.power, mean.u) annotation (Line(points={{-42,23},{-52,
              23},{-52,15.4},{-51,15.4}}, color={0,0,127}));
      connect(resistor.heatPort, Water.port)
        annotation (Line(points={{-4,6},{16,6},{16,16}}, color={191,0,0}));
      connect(temperatureSensor.port, Water.port)
        annotation (Line(points={{28,6},{16,6},{16,16}}, color={191,0,0}));
      connect(KettleWall.port_a, Water.port)
        annotation (Line(points={{16,-20},{16,16}}, color={191,0,0}));
      connect(KettleWall.port_b, RoomTemperature.port) annotation (Line(points=
              {{16,-40},{16,-46},{-2,-46}}, color={191,0,0}));
      connect(temperatureSensor.T, onOffController.u)
        annotation (Line(points={{48,6},{63,6}}, color={0,0,127}));
      connect(const.y, onOffController.reference) annotation (Line(points={{
              44.8,46},{54,46},{54,12},{63,12}}, color={0,0,127}));
      connect(powerSensor.pc, switch1.n)
        annotation (Line(points={{-42,34},{-58,34}}, color={0,0,255}));
      connect(sineVoltage.p, switch1.p) annotation (Line(points={{-80,10},{-76,
              10},{-76,34},{-70,34}}, color={0,0,255}));
      connect(onOffController.y, switch1.control) annotation (Line(points={{
              74.5,9},{74.5,76},{-64,76},{-64,41.2}}, color={255,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=6000, __Dymola_NumberOfIntervals=5000));
    end ElectricKettle;
  end Tutorial4;
  annotation (uses(Modelica(version="3.2.3")));
end MyWork;
