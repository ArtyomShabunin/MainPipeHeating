package MainPipeHeating
  model HeatingPipeTest_1
    replaceable package Medium = Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium;
    //Характеристики металла трубопровода
    parameter Modelica.SIunits.SpecificHeatCapacity Cp = 470 "Удельная теплоемкость стали(Дж/кг/К)";
    parameter Modelica.SIunits.Density rho = 7800 "Плотность стали (кг/м3)";
    //Геометрические характеристики трубопроводов
    parameter Modelica.SIunits.Diameter Dout = 0.273 "Наружный диаметр основного трубопровода (м)";
    parameter Modelica.SIunits.Thickness delta = 0.026 "Толщина стенки основного трубопровода (м)";
    //Параметры участков трубопроводов
    //Участок №1
    parameter Modelica.SIunits.Length Lpipe1 = 15 "Длина первого участка трубопровода (м)";
    parameter Modelica.SIunits.Area Cpipe1 = Cp * rho * Modelica.Constants.pi * (Dout ^ 2 - (Dout - 2 * delta) ^ 2) * Lpipe1 "Теплоемкость первого участка трубопровода (Дж/К)";
    //Участок №2
    parameter Modelica.SIunits.Length Lpipe2 = 8 "Длина второго участка трубопровода (м)";
    parameter Modelica.SIunits.Area Cpipe2 = Cp * rho * Modelica.Constants.pi * (Dout ^ 2 - (Dout - 2 * delta) ^ 2) * Lpipe2 "Теплоемкость первого участка трубопровода (Дж/К)";  
    //Номинальные параметры пара перед БРОУ
    parameter Modelica.SIunits.Pressure Pnom = 8e6 "Номинальное давление перед ПТ (Па)";
    parameter Modelica.SIunits.Temperature Tnom = 540 + 273.13 "Номинальная температура пара перед ПТ (К)";
    parameter Modelica.SIunits.MassFlowRate Dnom = 113.8 / 3.6 "Номинальная паропроизводительность (кг/с)";
    //Параметры при прогреве
    parameter Modelica.SIunits.Temperature Theat = 300 + 273.15 "Температура греющего пара (К)";
    parameter Modelica.SIunits.SpecificEnthalpy hheat = Medium.specificEnthalpy_pT(pheat, Theat) "Значение энтальпии входного потока при прогреве";  
    parameter Modelica.SIunits.Pressure pheat = 30e5 "Давление в трубопроводе при прогреве";
    parameter Modelica.SIunits.MassFlowRate Dheat = 0.5 / 3.6 "Расход греющего пара (кг/с)";
    parameter Modelica.SIunits.Temperature Tinit = Medium.saturationTemperature(pheat) "Исходная температура металла паропровода (К)";
    parameter Modelica.SIunits.SpecificEnthalpy hinit = Medium.dewEnthalpy(Medium.setSat_p(pheat)) "Начальное значение энтальпии входного потока";
    inner Modelica.Fluid.System system annotation(
      Placement(visible = true, transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Fluid.Pipes.DynamicPipe pipe1(redeclare package Medium = Medium, redeclare model HeatTransfer = Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.LocalPipeFlowHeatTransfer, diameter = Dout - 2 * delta, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyStateInitial, length = Lpipe1, massDynamics = Modelica.Fluid.Types.Dynamics.SteadyStateInitial, momentumDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, nNodes = 5, use_HeatTransfer = true) annotation(
      Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor[pipe1.nNodes] wall1(C = Cpipe1 / pipe1.nNodes * ones(pipe1.nNodes), der_T(fixed = true)) annotation(
      Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp ramp1(duration = 3, height = hheat - hinit, offset = hinit, startTime = 10)  annotation(
      Placement(visible = true, transformation(origin = {-90, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor wall2[pipe2.nNodes](C = Cpipe2 / pipe2.nNodes * ones(pipe2.nNodes), der_T(fixed = true)) annotation(
      Placement(visible = true, transformation(origin = {40, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Pipes.DynamicPipe pipe2(redeclare package Medium = Medium, redeclare model HeatTransfer = Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.LocalPipeFlowHeatTransfer, diameter = Dout - 2 * delta, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyStateInitial, length = Lpipe2, massDynamics = Modelica.Fluid.Types.Dynamics.SteadyStateInitial, momentumDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, nNodes = 5, use_HeatTransfer = true) annotation(
      Placement(visible = true, transformation(origin = {40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Fittings.TeeJunctionIdeal Junction1 annotation(
      Placement(visible = true, transformation(origin = {20, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Fluid.Fittings.GenericResistances.VolumeFlowRate Drenaj1(redeclare package Medium = Medium, a = 1000, b = 0)  annotation(
      Placement(visible = true, transformation(origin = {20, 18}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Fluid.Sources.Boundary_ph InletFlow(redeclare package Medium = Medium, nPorts = 1, use_h_in = true)  annotation(
      Placement(visible = true, transformation(origin = {-42, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Sources.MassFlowSource_h BROU(redeclare package Medium = Medium, m_flow = -Dheat, nPorts = 1) annotation(
      Placement(visible = true, transformation(origin = {90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Fluid.Sources.FixedBoundary OutletFlow(redeclare package Medium = Medium, nPorts = 1) annotation(
      Placement(visible = true, transformation(origin = {20, -32}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  equation
    connect(Drenaj1.port_b, OutletFlow.ports[1]) annotation(
      Line(points = {{20, 8}, {20, 8}, {20, -22}, {20, -22}}, color = {0, 127, 255}));
    connect(ramp1.y, InletFlow.h_in) annotation(
      Line(points = {{-78, 54}, {-54, 54}, {-54, 54}, {-54, 54}}, color = {0, 0, 127}));
    connect(InletFlow.ports[1], pipe1.port_a) annotation(
      Line(points = {{-32, 50}, {-10, 50}}, color = {0, 127, 255}));
    connect(pipe2.port_b, BROU.ports[1]) annotation(
      Line(points = {{50, 50}, {90, 50}, {90, -20}, {90, -20}}, color = {0, 127, 255}));
    connect(Junction1.port_3, Drenaj1.port_a) annotation(
      Line(points = {{20, 40}, {20, 40}, {20, 28}, {20, 28}}, color = {0, 127, 255}));
    connect(wall2.port, pipe2.heatPorts) annotation(
      Line(points = {{40, 70}, {40, 54}}, color = {191, 0, 0}));
    connect(Junction1.port_1, pipe2.port_a) annotation(
      Line(points = {{30, 50}, {30, 50}}, color = {0, 127, 255}));
    connect(Junction1.port_2, pipe1.port_b) annotation(
      Line(points = {{10, 50}, {10, 50}}, color = {0, 127, 255}));
    connect(wall1.port, pipe1.heatPorts) annotation(
      Line(points = {{0, 70}, {0, 54}}, color = {191, 0, 0}));
  end HeatingPipeTest_1;





































  annotation(
    uses(Modelica(version = "3.2.2")));
end MainPipeHeating;
