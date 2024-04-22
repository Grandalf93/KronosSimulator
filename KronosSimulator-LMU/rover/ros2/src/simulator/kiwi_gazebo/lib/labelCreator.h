#ifndef LABELCREATOR_H
#define LABELCREATOR_H

#include "tinyxml2.h"
#include <string>
#include <ignition/math/Vector3.hh>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>





/**
 * @class labelCreator
 * 
 * Implementation of a XML parser 
 *
 * XML label generator. It parses a XML document through the library tinyxml2.
 * The library is of the form tinyxml2::<Class> where >Class> could be either:
 * XMLDocument, XMLElement, XMLAttribute, XMLNode. The document is of the form:
 * <root>
 *  <child = /attributeValue/ >
 *      <subchild> /text/ </subchild>
 *  </child>
 * </root>
 *  
 * @see [tinyxml2](https://docs.scipy.org/doc/numpy/reference/generated/numpy.vectorize.html)  * 
 */


class labelCreator{    
    

    public:

    /**
     * Create a new XML label in the document 
     * 
     * @param  doc A reference of the document to be parsed. 
     * @param  parent A pointer to the parent of the element to be created.
     * @param  element Name of the label 
     * @param  text text content of the element. Pass "\n" if a new line has to be inserted. That is, a subchild structure
     */    
    static tinyxml2::XMLElement * InsertNewLabel(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent  ,const char* element, const char* text);

    /**
     * Create a new XML label in the document 
     * 
     * @param  doc A reference of the document to be parsed. 
     * @param  parent A pointer to the parent of the element to be created.
     * @param  element Name of the label/node 
     * @param  text text content of the element. Pass "\n" if a new line has to be inserted. That is, a subchild structure
     * @param  nameAttribute name of the attribute of the element
     * @param  attributeValue text content of the attribute
     * 
     * @overload
     */ 
    static tinyxml2::XMLElement * InsertNewLabel(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent  ,const char* element, const char* text, const char* nameAttribute, const char* attributeValue);

    /**
     * Create a new XML label in the document 
     * 
     * @param  doc A reference of the document to be parsed. 
     * @param  parent A pointer to the parent of the element to be created.
     * @param  element Name of the label/node 
     * @param  nameAttribute name of the attribute of the element
     * @param  attributeValue text content of the attribute
     * 
     * @overload
     */ 
    static tinyxml2::XMLElement * InsertNewLabel(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent  ,const char* element, const char* nameAttribute, const char* attributeValue);

    /**
     * Create a new XML label in the document 
     * 
     * @param  doc A reference of the document to be parsed. Document created by class XMLDocument
     * @param  parent A pointer to the parent of the element to be created.
     * @param  element Name of the label/node 
     * 
     * @overload
     */     
    static tinyxml2::XMLElement * InsertNewLabel(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const char* element );



    /**
     * Create a new XML Element in the document. That is, the root element. Call this method to handle the parser
     * 
     * @param  NewDoc A reference of the document to be parsed. Document created by class XMLDocument
     * @param  name class of the element i.e. Sidewalk, Grass, Oak_Tree
     * @param  points set of data of the form [X,Y,Z] coordinates of the model to be spawned on the map
     * 
     * @return string-type message with alphanumeric identifier of the model
     * 
     * @overload
     */ 
    static std::string  NewXmlModel(tinyxml2::XMLDocument &NewDoc, const char* name, std::vector<ignition::math::Vector3d> points);


    /**
     * Loops through the points/vertex or coordinates of the element. It sets height and position
     * 
     * @param  xmlDoc A reference of the document to be parsed. Document created by class XMLDocument
     * @param  elementPine class of the element i.e. Sidewalk, Grass, Oak_Tree
     * @param  points set of data of the form [X,Y,Z] coordinates of the model to be spawned on the map
     * 
     */ 
    static void  PointIterator(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *elementPine, const char* entity, std::vector<ignition::math::Vector3d> points);


    /**
     * Creates the xml body of polyline models
     * 
     * @param  xmlDoc A reference of the document to be parsed. Document created by class XMLDocument
     * @param  elementPine class of the element i.e. Sidewalk, Grass, Oak_Tree
     * @param  points set of data of the form [X,Y,Z] coordinates of the model to be spawned on the map
     * 
     */  
    static void  PolygonBodyConstruction(tinyxml2::XMLDocument &NewDoc,tinyxml2::XMLElement *pRoot, const char* entity, std::vector<ignition::math::Vector3d> points);

    /**
     * This method generates a random alphanumeric hash of a desired length 
     * 
     * @param  len Lenght of the hash to be generated
     */      
    static std::string  gen_random(const int len);

    /**
     * It sets the color of the model
     * 
     * @param  doc A reference of the document to be parsed. 
     * @param  parent A pointer to the parent of the created element.
     * @param  decision class of the element i.e. Sidewalk, Grass, Oak_Tree
     */  
    static tinyxml2::XMLElement *  ambientColor(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, std::string decision);

    /**
     * Creates the xml body of the tree model
     * 
     * @param  xmlDoc A reference of the document to be parsed. Document created by class XMLDocument
     * @param  pRoot A pointer to the parent of the created element.
     * @param  points set of data of the form [X,Y,Z] coordinates of the model to be spawned on the map
     */  
    static void TreeBodyConstruction(tinyxml2::XMLDocument &NewDoc, tinyxml2::XMLElement *pRoot, std::vector<ignition::math::Vector3d> points);


    /**
     * Creates the intermediate body of the tree body. It sets submesh values
     * 
     * @param  NewDoc A reference of the document to be parsed. Document created by class XMLDocument
     * @param  parent A pointer to the parent of the created element.
     * @param  submeshValue sumbmesh name
     * @param  nameValue  name of the submesh
     */  
    static void IntermediateConstruction (tinyxml2::XMLDocument &NewDoc, tinyxml2::XMLElement *parent, const char* submeshValue, const char* nameValue);



};

#endif