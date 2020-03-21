<?xml version="1.0" encoding="utf-8"?>
<xsl:stylesheet version="1.0"
                xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
                xmlns:sc="http://www.w3.org/2005/07/scxml">

  <xsl:output method="text" encoding="utf-8"/>
  <xsl:strip-space elements="*" />
  <xsl:template match="text()|@*"/>
  <xsl:template match="text()|@*" mode="target"/>
  <xsl:template match="text()|@*" mode="id"/>

  <xsl:template match="/sc:scxml">
    <xsl:text>@startuml&#xa;</xsl:text>
    <xsl:apply-templates select="@initial"/>
    <xsl:apply-templates select="*"/>
    <xsl:text>@enduml&#xa;</xsl:text>
  </xsl:template>

  <xsl:template match="@initial">
    <xsl:text>[*] --> </xsl:text>
    <xsl:call-template name="gettargetname">
      <xsl:with-param name="id" select="."/>
    </xsl:call-template>
    <xsl:text>&#xa;</xsl:text>
  </xsl:template>

  <xsl:template match="sc:initial">
    <xsl:apply-templates select="*"/>
  </xsl:template>

  <xsl:template match="sc:final[@id]">
    <xsl:value-of select="concat(@id, ' : &lt;&lt;final>>&#xa;')"/>
  </xsl:template>
  <xsl:template match="sc:final">
    <xsl:text>unnamed : &lt;&lt;final>>&#xa;</xsl:text>
  </xsl:template>

  <xsl:template match="sc:state">
    <xsl:text>state </xsl:text>
    <xsl:apply-templates select="." mode="id"/>
    <xsl:text> {&#xa;</xsl:text>

    <xsl:apply-templates select="@initial"/>
    <xsl:apply-templates select="*"/>
    
    <xsl:text>}&#xa;</xsl:text>
  </xsl:template>

  <xsl:template match="sc:history">
    <xsl:text>state </xsl:text>
    <xsl:apply-templates select="." mode="id"/>
    <xsl:text> {&#xa;</xsl:text>
    
    <xsl:apply-templates select="*"/>

    <xsl:apply-templates select="." mode="id"/>
    <xsl:value-of select="concat(' : &lt;&lt;', @type, ' history>>&#xa;')"/>

    <xsl:text>}&#xa;</xsl:text>
  </xsl:template>

  <xsl:template match="sc:parallel">
    <xsl:text>state </xsl:text>
    <xsl:apply-templates select="." mode="id"/>
    <xsl:text> {&#xa;</xsl:text>

    <xsl:for-each select="sc:state | sc:parallel | sc:history">
      <xsl:apply-templates select="."/>
      <xsl:if test="position() != last()">
        <xsl:text>||&#xa;</xsl:text>
      </xsl:if>
    </xsl:for-each>

    <xsl:apply-templates select="sc:transition"/>

    <xsl:text>}&#xa;</xsl:text>
  </xsl:template>


  <!--get id of the state - special case for initial state-->
  <xsl:template match="sc:state[@id] | sc:history[@id] | sc:parallel[@id] | sc:final[@id]" mode="id">
    <xsl:value-of select="@id"/>
  </xsl:template>
  <xsl:template match="sc:state | sc:history | sc:parallel" mode="id">
    <xsl:text>unnamed</xsl:text>
  </xsl:template>
  <xsl:template match="sc:initial" mode="id">
    <xsl:text>[*]</xsl:text>
  </xsl:template>

  <xsl:template name="gettargetname">
    <xsl:param name="id"/>
    <xsl:apply-templates select="//*[@id = $id][1]" mode="id"/>
  </xsl:template>

  <!--target can be specified explicitely as some other state-->
  <xsl:template match="sc:transition[@target]" mode="target">
    <xsl:call-template name="gettargetname">
      <xsl:with-param name="id" select="@target"/>
    </xsl:call-template>
  </xsl:template>
  <!--or self-transiton when @target not specified-->
  <xsl:template match="sc:transition" mode="target">
    <xsl:apply-templates select=".." mode="id"/>
  </xsl:template>

  <!--if 'cond' attribute exists it shall be sourrounded with brackets-->
  <xsl:template match="sc:transition/@cond">
    <xsl:value-of select="concat('[', ., ']')"/>
  </xsl:template>

  <xsl:template match="sc:transition">
    <xsl:apply-templates select=".." mode="id"/>
    <xsl:text> --> </xsl:text>
    <xsl:apply-templates select="." mode="target"/>
    <xsl:if test="@event or @cond">
      <xsl:text> : </xsl:text>
    </xsl:if>
    <xsl:value-of select="@event"/>
    <xsl:apply-templates select="@cond"/>
    <xsl:text>&#xa;</xsl:text>
  </xsl:template>

</xsl:stylesheet>
