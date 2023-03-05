
import time
import math
from datetime import datetime

class Record:
    def __init__(self):
        self.agentNo=0
        self.agentID=0
        self.agentType=None
        self.agentTypeNo=None

        self.x = None
        self.y= None
        self.z= None
        self.yaw= None
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.speed = 0
        self.time = 0
        self.sim_time=0


class Parser:
    def __init__(self):
        self.logs = []
        self.agent_keys=set()
        self.agentNo_R = {}
        self.path = ""

    def setInput(self,tpath):
        self.path = tpath

    def readFile(self):
        f = open(self.path,mode='r', encoding='utf-8')

        while True:
            line = f.readline()
            line.strip()
            if line is None or line == '':
                break
            split_letters = line.split(',')
            record = Record()
            record.agentNo = int(split_letters[2])
            record.agentID = int(split_letters[3])
            record.agentType = split_letters[4]
            record.agentTypeNo = int(split_letters[5])
            record.x = float(split_letters[6])
            record.y = float(split_letters[7])
            record.z = float(split_letters[8])
            record.yaw = float(split_letters[9])
            record.vel_x = float(split_letters[10])
            record.vel_y = float(split_letters[11])
            record.vel_z = float(split_letters[12])
            record.speed = float(split_letters[13])
            record.time = float(split_letters[14])
            record.sim_time = float(split_letters[15])

            agentNo = record.agentNo
            self.agent_keys.add(agentNo)
            if agentNo in self.agentNo_R.keys():
                self.agentNo_R[agentNo].append(record)
            else:
                self.agentNo_R[agentNo] = []
                self.agentNo_R[agentNo].append(record)
            self.logs.append(record)

    def clear(self):
        self.logs.clear()

class Writer:
    def __init__(self):
        self.velocities = []
        self.wayPoints = []
        self.count = 0
        self.f = None
        self.parser = Parser()
        self.cur_ref = 0
        self.lookhead = 0.6
        self.path = ""

    def setInput(self,path):
        self.parser.setInput(path)

    def setOutput(self,tpath):
        self.path = tpath

    def eventActions(self,agentNum):
        self.f.write("      <Event name =\"RouteEvent{}\" priority=\"overwrite\">".format(self.count))
        self.f.write("\n")
        self.f.write("       <Action name =\"Assign Route\">")
        self.f.write("\n")
        self.f.write("        <PrivateAction>")
        self.f.write("\n")
        self.f.write("         <RoutingAction>")
        self.f.write("\n")
        self.f.write("          <AssignRouteAction>")
        self.f.write("\n")
        self.f.write("           <Route name=\"Route 1\" closed = \"false\">")
        self.f.write("\n")
        self.mapWayPoints(agentNum)
        self.f.write("           </Route>")
        self.f.write("\n")
        self.f.write("          </AssignRouteAction>")
        self.f.write("\n")
        self.f.write("         </RoutingAction>")
        self.f.write("\n")
        self.f.write("        </PrivateAction>")
        self.f.write("\n")
        self.f.write("       </Action>")
        self.f.write("\n")
        self.f.write("       <StartTrigger>")
        self.f.write("\n")
        self.f.write("        <ConditionGroup>")
        self.f.write("\n")
        self.f.write("         <Condition name=\"\" delay=\"0\" conditionEdge=\"rising\">")
        self.f.write("\n")
        self.f.write("          <ByValueCondition>")
        self.f.write("\n")
        self.f.write("           <SimulationTimeCondition value=\"0\" rule=\"greaterThan\"/>")
        self.f.write("\n")
        self.f.write("          </ByValueCondition>")
        self.f.write("\n")
        self.f.write("         </Condition>")
        self.f.write("\n")
        self.f.write("        </ConditionGroup>")
        self.f.write("\n")
        self.f.write("       </StartTrigger>")
        self.f.write("\n")
        self.f.write("      </Event>")
        self.f.write("\n")
        self.setSpeedEvents(agentNum)

    def setSpeedEvents(self,agentNum):
        index = 0
        for veloctity in self.velocities:
            self.f.write("      <Event name =\"RouteEvent{}\" priority=\"parallel\">".format(self.count))
            self.f.write("\n")
            self.f.write("       <Action name =\"ActionSpeed{}\">".format(self.count))
            self.f.write("\n")
            self.f.write("        <PrivateAction>")
            self.f.write("\n")
            self.f.write("         <LongitudinalAction>")
            self.f.write("\n")
            self.f.write("          <SpeedAction>")
            self.f.write("\n")
            self.f.write("           <SpeedActionDynamics dynamicsShape=\"step\" value=\"0\" dynamicsDimension=\"time\"/>")
            self.f.write("\n")
            self.f.write("           <SpeedActionTarget>")
            self.f.write("\n")
            self.f.write("            <AbsoluteTargetSpeed value=\"{}\"/>".format(veloctity))
            self.f.write("\n")
            self.f.write("           </SpeedActionTarget>")
            self.f.write("\n")
            self.f.write("          </SpeedAction>")
            self.f.write("\n")
            self.f.write("         </LongitudinalAction>")
            self.f.write("\n")
            self.f.write("        </PrivateAction>")
            self.f.write("\n")
            self.f.write("       </Action>")

            self.f.write("\n")
            self.f.write("       <StartTrigger>")
            self.f.write("\n")
            self.f.write("        <ConditionGroup>")
            self.f.write("\n")
            self.f.write("         <Condition name=\"\" delay=\"0\" conditionEdge=\"none\">")
            self.f.write("\n")
            self.f.write("          <ByEntityCondition>")
            self.f.write("\n")
            self.f.write("           <TriggeringEntities triggeringEntitiesRule=\"any\">")
            self.f.write("\n")
            self.f.write("            <EntityRef entityRef=\"{}\"/>".format(agentNum))
            self.f.write("\n")
            self.f.write("           </TriggeringEntities>")
            self.f.write("\n")
            self.f.write("           <EntityCondition>")
            self.f.write("\n")
            self.f.write("            <ReachPositionCondition tolerance=\"2\">")
            self.f.write("\n")
            self.f.write("             <Position>")
            self.f.write("\n")
            waypoint = self.wayPoints[index]
            self.f.write("              <WorldPosition x=\"{}\" y=\"{}\" z=\"{}\"/>".format(waypoint.x,waypoint.y,waypoint.z))
            self.f.write("\n")
            self.f.write("             </Position>")
            self.f.write("\n")
            self.f.write("            </ReachPositionCondition>")
            self.f.write("\n")
            self.f.write("           </EntityCondition>")
            self.f.write("\n")
            self.f.write("          </ByEntityCondition>")
            self.f.write("\n")
            self.f.write("         </Condition>")
            self.f.write("\n")
            self.f.write("        </ConditionGroup>")
            self.f.write("\n")
            self.f.write("       </StartTrigger>")
            self.f.write("\n")
            self.f.write("      </Event>")
            self.f.write("\n")
            self.count = self.count + 1
            index = index + 1
        self.velocities.clear();
        self.wayPoints.clear();

    def storyManeuvers(self):
        index = 0
        for agentNo in self.parser.agent_keys:
            self.f.write("    <ManeuverGroup name=\"Group{}\" maximumExecutionCount=\"1\">".format(index))
            self.f.write("\n")
            self.f.write("     <Actors selectTriggeringEntities=\"false\">")
            self.f.write("\n")
            self.f.write("      <EntityRef entityRef=\"{}\"/>".format(agentNo))
            self.f.write("\n")
            self.f.write("     </Actors>")
            self.f.write("\n")
            self.f.write("     <Maneuver name=\"FollowWaypoints{}\">".format(index))
            self.f.write("\n")
            self.eventActions(agentNo)
            self.f.write("     </Maneuver>")
            self.f.write("\n")
            self.f.write("    </ManeuverGroup>")
            self.f.write("\n")
            index = index + 1


    def storyInit(self):
        self.f.write("  <Story name=\"MainStory\">")
        self.f.write("\n")
        self.f.write("   <Act name =\"MainAct\">")
        self.f.write("\n")
        self.storyManeuvers()
        self.f.write("    <StartTrigger>")
        self.f.write("\n")
        self.f.write("     <ConditionGroup>")
        self.f.write("\n")
        self.f.write("      <Condition name=\"\" delay=\"0\" conditionEdge=\"rising\">")
        self.f.write("\n")
        self.f.write("       <ByValueCondition>")
        self.f.write("\n")
        self.f.write("        <SimulationTimeCondition value=\"0\" rule=\"greaterThan\"/>")
        self.f.write("\n")
        self.f.write("       </ByValueCondition>")
        self.f.write("\n")
        self.f.write("      </Condition>")
        self.f.write("\n")
        self.f.write("     </ConditionGroup>")
        self.f.write("\n")
        self.f.write("    </StartTrigger>")
        self.f.write("\n")
        self.f.write("   </Act>")
        self.f.write("\n")
        self.f.write("  </Story>")
        self.f.write("\n")
        self.f.write("  <StopTrigger>")
        self.f.write("\n")
        self.f.write("   <ConditionGroup>")
        self.f.write("\n")
        self.f.write("    <Condition name=\"\" delay=\"0\" conditionEdge=\"rising\">")
        self.f.write("\n")
        self.f.write("     <ByValueCondition>")
        self.f.write("\n")
        self.f.write("      <SimulationTimeCondition value=\"30\" rule=\"greaterThan\"/>")
        self.f.write("\n")
        self.f.write("     </ByValueCondition>")
        self.f.write("\n")
        self.f.write("    </Condition>")
        self.f.write("\n")
        self.f.write("   </ConditionGroup>")
        self.f.write("\n")
        self.f.write("  </StopTrigger>")
        self.f.write("\n")
        self.f.write(" </Storyboard>")
        self.f.write("\n")
        self.f.write("</OpenSCENARIO>")

    def initActions(self):
        self.f.write(" <Storyboard>")
        self.f.write("\n")
        self.f.write("  <Init>")
        self.f.write("\n")
        self.f.write(("   <Actions>"))
        self.f.write("\n")
        self.writeWeather()
        self.f.write("\n")
        index = 0
        for agentNo in self.parser.agent_keys:
            self.f.write("    <Private entityRef=\"{}\">".format(index+1))
            self.f.write("\n")
            if agentNo==2:
                self.writeControl()
            self.f.write("     <PrivateAction>")
            self.f.write("\n")
            self.f.write("      <TeleportAction>")
            self.f.write("\n")
            self.f.write("       <Position>")
            self.f.write("\n")
            initRecord = self.parser.agentNo_R[agentNo][0]
            self.f.write("        <WorldPosition x=\"{}\" y=\"{}\" z=\"{}\" h=\"{}\"/>".format(initRecord.x, initRecord.y,initRecord.z,3.14159))  #initRecord.yaw
            self.f.write("\n")
            self.f.write("       </Position>")
            self.f.write("\n")
            self.f.write("      </TeleportAction>")
            self.f.write("\n")
            self.f.write("     </PrivateAction>")
            self.f.write("\n")
            self.f.write("    </Private>")
            self.f.write("\n")
            index = index + 1
        self.f.write("   </Actions>")
        self.f.write("\n")
        self.f.write("  </Init>")
        self.f.write("\n")



    def classifyObject(self):
        for agentNo in self.parser.agent_keys:
            self.f.write("  <ScenarioObject name=\"{}\">".format(agentNo))
            self.f.write("\n")
            self.f.write("   <Vehicle name=\"{}\" vehicleCategory=\"car\">".format(self.parser.agentNo_R[agentNo][0].agentType))
            self.f.write("\n")
            self.f.write("    <ParameterDeclarations/>")
            self.f.write("\n")
            self.f.write("    <Performance maxSpeed=\"69.444\" maxAcceleration=\"200\" maxDeceleration=\"10.0\"/>")
            self.f.write("\n")
            self.f.write("    <BoundingBox>")
            self.f.write("\n")
            self.f.write("     <Center x=\"1.5\" y=\"0.0\" z=\"0.9\"/>")
            self.f.write("\n")
            self.f.write("     <Dimensions width= \"2.1\" length= \"4.5\" height=\"1.8\"/>")
            self.f.write("\n")
            self.f.write("    </BoundingBox>")
            self.f.write("\n")
            self.f.write("    <Axles>")
            self.f.write("\n")
            self.f.write("     <FrontAxle maxSteering=\"0.5\" wheelDiameter=\"0.6\" trackWidth=\"1.8\" positionX=\"3.1\" positionZ=\"0.3\"/>")
            self.f.write("\n")
            self.f.write("     <RearAxle maxSteering=\"0.0\" wheelDiameter=\"0.6\" trackWidth=\"1.8\" positionX=\"0.0\" positionZ=\"0.3\"/>")
            self.f.write("\n")
            self.f.write("    </Axles>")
            self.f.write("\n")
            self.f.write("    <Properties>")
            self.f.write("\n")
            self.f.write("     <Property name=\"type\" value=\"vehicle{}\"/>".format(agentNo))
            self.f.write("\n")
            self.f.write("    </Properties>")
            self.f.write("\n")
            self.f.write("   </Vehicle>");
            self.f.write("\n")
            # } else {
            # self.f.write("   <Pedestrian model=\"" + logLines.get(i).getAgentType() + "\" mass =\"90\" name=\"" + logLines.get(i).getAgentType() + "\" pedestrianCategory=\"pedestrian\">");
            # self.f.write(\n);
            # self.f.write("    <ParameterDeclarations/>");
            # self.f.write(\n);
            # self.f.write("    <BoundingBox>");
            # self.f.write(\n);
            # self.f.write("     <Center x=\"1.5\" y=\"0.0\" z=\"0.9\"/>");
            # self.f.write(\n);
            # self.f.write("     <Dimensions width=\"2.1\" length=\"4.5\" height=\"1.8\"/>");
            # self.f.write(\n);
            # self.f.write("    </BoundingBox>");
            # self.f.write(\n);
            # self.f.write("    <Properties>");
            # self.f.write(\n);
            # self.f.write("     <Property name=\"type\" value=\"pedestrian" + i + "\"/>");
            # self.f.write(\n);
            # self.f.write("    </Properties>");
            # self.f.write(\n);
            # self.f.write("   </Pedestrian>");
            # self.f.write(\n);
            # }
            self.f.write("  </ScenarioObject>");

    def writeInEntities(self):
        self.f.write(" <Entities>")
        self.f.write("\n")
        self.classifyObject()
        self.f.write(" </Entities>")
        self.f.write("\n")

    def writeTopLines(self,pathToMap):
        self.f.write('<?xml version=\"1.0\" encoding=\"UTF-8\"?>')
        self.f.write('\n')
        self.f.write('<OpenSCENARIO xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"OpenSCENARIO.xsd\">')
        self.f.write('\n')
        self.f.write(" <FileHeader revMajor=\"1\" revMinor=\"0\" date=\"2023-02-10T16:27:29.575742\" description=\"CARLA:Converted from logs\" author=\"Efan Haynes\"/>")
        self.f.write("\n")
        self.f.write(' <ParameterDeclarations/>')
        self.f.write("\n")

        self.f.write(' <CatalogLocations/>')
        self.f.write("\n")
        self.f.write(' <RoadNetwork>')
        self.f.write("\n")
        self.f.write('  <LogicFile filepath=\"{}\"/>'.format(pathToMap))
        self.f.write("\n")
        self.f.write(' </RoadNetwork>')
        self.f.write("\n")

    def writeWeather(self):
        self.f.write('              <GlobalAction>  \n \
               <EnvironmentAction>   \n              \
                <Environment name=\"Environment1\"> \n \
                  <TimeOfDay animation=\"false\" dateTime=\"2020-03-20T12:00:00\"/> \n \
                  <Weather cloudState="free">    \n                                \
                    <Sun intensity="0.85" azimuth=\"0\" elevation=\"1.31\"/>  \n       \
                    <Fog visualRange="100000.0"/>               \n                 \
                    <Precipitation precipitationType="dry" intensity=\"0.0\"/>  \n   \
                  </Weather>                                      \n               \
                  <RoadCondition frictionScaleFactor=\"1.0\"/>   \n                  \
                </Environment>                                  \n                 \
               </EnvironmentAction>                      \n                        \
            </GlobalAction>')
        self.f.write("\n")

    def writeControl(self):
        self.f.write('                    <PrivateAction>\n                           \
                        <ControllerAction>  \n                                        \
                            <OverrideControllerValueAction>  \n                       \
                                <Throttle active=\"false\" value=\"0.0\"/>  \n        \
                                <Brake active=\"false\" value=\"0.0\"/>  \n           \
                                <Clutch active=\"false\" value=\"0.0\"/>  \n          \
                                <ParkingBrake active=\"false\" value=\"0.0\"/> \n     \
                                <SteeringWheel active=\"false\" value=\"0.0\"/> \n    \
                                <Gear active=\"false\" number=\"0.0\"/>  \n           \
                            </OverrideControllerValueAction>  \n                      \
                            <AssignControllerAction>    \n                            \
                                <Controller name=\"Camera\">   \n                     \
                                    <ParameterDeclarations/> \n                       \
                                    <Properties>       \n                             \
                                        <Property name=\"module\" value=\"simple_vehicle_control\"/>  \n   \
                                        <Property name=\"attach_camera\" value=\"true\"/>   \n            \
                                    </Properties>   \n                                                    \
                                </Controller>     \n                                                      \
                            </AssignControllerAction>   \n                                                \
                        </ControllerAction>      \n                                                       \
                    </PrivateAction>')
        self.f.write("\n")

    def writeFile(self):
        self.f = open(self.path, mode='w', encoding='utf-8')
        self.parser.readFile()
        self.writeTopLines('testme_signals')
        self.writeInEntities()
        self.initActions()
        self.storyInit()

    def calculateAverage(self,source,destination,Option):
        xDiff = source.x - destination.x
        yDiff = source.y - destination.y
        # zDiff = source.z - destination.z
        # (zDiff * zDiff) +
        distanceChange = math.sqrt( (yDiff * yDiff) + (xDiff * xDiff));
        if Option == "Distance":
            return distanceChange

        timeDiff = destination.sim_time - source.sim_time

        avgSpeed = distanceChange / timeDiff
        return avgSpeed

    def mapWayPoints(self,agentNum):

        records = self.parser.agentNo_R[agentNum]
        # self.wayPoints.append(records[0])
        src_index = 0
        dst_index = 1
        while True:
            if src_index >= (len(records)-1) or dst_index >= len(records):
                break
            source = records[src_index]
            dest = records[dst_index]
            distance = self.calculateAverage(source, dest, 'Distance')
            if distance >= self.lookhead:
                vel = self.calculateAverage(source, dest, 'Speed')
                self.velocities.append(vel)
                self.wayPoints.append(source)
                self.f.write("            <Waypoint routeStrategy=\"shortest\">")
                self.f.write("\n")
                self.f.write("             <Position>")
                self.f.write("\n")
                self.f.write(
                    "              <WorldPosition x=\"{}\" y=\"{}\" z=\"{}\" h=\"{}\"/>".format(source.x, source.y,
                                                                                                source.z,
                                                                                                source.yaw))
                self.f.write("\n")
                self.f.write("             </Position>")
                self.f.write("\n")
                self.f.write("            </Waypoint>")
                self.f.write("\n")
                src_index = dst_index
            dst_index = dst_index + 1



if __name__ == "__main__":
    w = Writer()
    w.setInput("demo.txt")
    w.setOutput('output.txt')
    w.writeFile()

