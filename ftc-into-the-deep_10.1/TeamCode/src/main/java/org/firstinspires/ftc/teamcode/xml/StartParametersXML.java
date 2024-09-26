package org.firstinspires.ftc.teamcode.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

public class StartParametersXML {

    public static final String TAG = StartParametersXML.class.getSimpleName();
    private static final String FILE_NAME = "StartParameters.xml";
    private final String xmlDirectory;
    private final String xmlFilePath;
    private final Document document;

    private final Node delay_node; // save the DOM Node for update
    private int autoStartDelay;

    private final StartParameters startParameters;

    // IntelliJ only
    /*
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */
    // End IntelliJ only

    public StartParametersXML(String pXMLDirectory) throws ParserConfigurationException, SAXException, IOException {

        // IntelliJ only
        /*
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser.
        dbFactory.setIgnoringElementContentWhitespace(true);
         */
        // End IntelliJ only

        // Android only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        // ONLY works with a validating parser (DTD or schema) dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
        // End Android only

        xmlDirectory = pXMLDirectory;
        xmlFilePath = xmlDirectory + FILE_NAME;

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        document = dBuilder.parse(new File(xmlFilePath));
        Element startParametersRoot = document.getDocumentElement();

        Node config_node = startParametersRoot.getFirstChild();
        config_node = XMLUtils.getNextElement(config_node);
        if (config_node == null || !config_node.getNodeName().equals("robot_config") || config_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'robot_config' not found");

        String robotConfigFilename = xmlDirectory + config_node.getTextContent();

        Node action_node = config_node.getNextSibling();
        action_node = XMLUtils.getNextElement(action_node);
        if (action_node == null || !action_node.getNodeName().equals("robot_action") || action_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'robot_action' not found");

        String robotActionFilename = xmlDirectory + action_node.getTextContent();

        Node local_delay_node = action_node.getNextSibling();
        local_delay_node = XMLUtils.getNextElement(local_delay_node);
        if (local_delay_node == null || !local_delay_node.getNodeName().equals("auto_start_delay") || local_delay_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'auto_start_delay' not found");

        String delayString = local_delay_node.getTextContent();
        try {
            autoStartDelay = Integer.parseInt(delayString);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'auto_start_delay'");
        }

        delay_node = local_delay_node; // preserve "final"

        startParameters = new StartParameters(robotConfigFilename, robotActionFilename, autoStartDelay);
        RobotLogCommon.c(TAG, "In StartParametersXML; opened and parsed the XML file");
    }

    // Warning: this method returns the start parameters from the XML
    // file. Any changes made to the parameters by any "set" methods
    // below are not reflected in the startParameters variable. If you
    // do want to reflect the changes you will have to include:
    //  startParameters = new StartParameters(robotConfigFilename, robotActionFilename, autoStartDelay, autoEndingPositions);
    // after every change.
    public StartParameters getStartParameters() {
        return startParameters;
    }

    // Replaces the text value of the <auto_start_delay> element.
    public void setAutoStartDelay(int pAutoStartDelay) {
        RobotLogCommon.c(TAG, "Setting the Autonomous start delay in startParameters to " + pAutoStartDelay);
        autoStartDelay = pAutoStartDelay;
        delay_node.setTextContent(Integer.toString(pAutoStartDelay));
    }

    public void writeStartParametersFile() {
        XMLUtils.writeXMLFile(document, xmlFilePath, xmlDirectory + RobotConstants.XSLT_FILE_NAME);
    }

}
