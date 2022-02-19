<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns:atom="http://www.w3.org/2005/Atom">
<xsl:output method="html" encoding="utf-8" />
<xsl:template match="/atom:feed">
	<xsl:text disable-output-escaping="yes">&lt;!DOCTYPE html &gt;</xsl:text>
	<html>
	<head>
		<xsl:text disable-output-escaping="yes"><![CDATA[
		<meta charset="utf-8">
	<meta name="viewport" content="width=device-width, initial-scale=1.0">
	<title>Atom Feed (Styled)</title>

    <link rel="stylesheet" type="text/css" href="http://localhost:4000/Race/assets/css/styles_feeling_responsive.css">

  

	<script src="http://localhost:4000/Race/assets/js/modernizr.min.js"></script>

	<script src="https://ajax.googleapis.com/ajax/libs/webfont/1.5.18/webfont.js"></script>
	<script>
		WebFont.load({
			google: {
				families: [ 'Lato:400,700,400italic:latin', 'Volkhov::latin' ]
			}
		});
	</script>

	<noscript>
		<link href='http://fonts.googleapis.com/css?family=Lato:400,700,400italic%7CVolkhov' rel='stylesheet' type='text/css'>
	</noscript>


	<!-- Search Engine Optimization -->
	<meta name="description" content="Generalized RAcing Intelligence Competition (GRAIC), is a synthesis competition for intelligent racing.">
	<meta name="google-site-verification" content="Vk0IOJ2jwG_qEoG7fuEXYqv0m2rLa8P778Fi_GrsgEQ">
	<meta name="msvalidate.01" content="0FB4C028ABCF07C908C54386ABD2D97F" >
	
	
	
	<link rel="canonical" href="http://localhost:4000/Race/assets/xslt/atom.xslt">


	<!-- Facebook Open Graph -->
	<meta property="og:title" content="Atom Feed (Styled)">
	<meta property="og:description" content="Generalized RAcing Intelligence Competition (GRAIC), is a synthesis competition for intelligent racing.">
	<meta property="og:url" content="http://localhost:4000/Race/assets/xslt/atom.xslt">
	<meta property="og:locale" content="en_EN">
	<meta property="og:type" content="website">
	<meta property="og:site_name" content="GRAIC">
	
	<meta property="article:author" content="https://www.facebook.com/phlow.media">


	
	<!-- Twitter -->
	<meta name="twitter:card" content="summary">
	<meta name="twitter:site" content="phlow">
	<meta name="twitter:creator" content="phlow">
	<meta name="twitter:title" content="Atom Feed (Styled)">
	<meta name="twitter:description" content="Generalized RAcing Intelligence Competition (GRAIC), is a synthesis competition for intelligent racing.">
	
	

	<link type="text/plain" rel="author" href="http://localhost:4000/Race/humans.txt">

	

	

	<link rel="icon" sizes="32x32" href="http://localhost:4000/Race/assets/img/favicon-32x32.png">

	<link rel="icon" sizes="192x192" href="http://localhost:4000/Race/assets/img/touch-icon-192x192.png">

	<link rel="apple-touch-icon-precomposed" sizes="180x180" href="http://localhost:4000/Race/assets/img/apple-touch-icon-180x180-precomposed.png">

	<link rel="apple-touch-icon-precomposed" sizes="152x152" href="http://localhost:4000/Race/assets/img/apple-touch-icon-152x152-precomposed.png">

	<link rel="apple-touch-icon-precomposed" sizes="144x144" href="http://localhost:4000/Race/assets/img/apple-touch-icon-144x144-precomposed.png">

	<link rel="apple-touch-icon-precomposed" sizes="120x120" href="http://localhost:4000/Race/assets/img/apple-touch-icon-120x120-precomposed.png">

	<link rel="apple-touch-icon-precomposed" sizes="114x114" href="http://localhost:4000/Race/assets/img/apple-touch-icon-114x114-precomposed.png">

	
	<link rel="apple-touch-icon-precomposed" sizes="76x76" href="http://localhost:4000/Race/assets/img/apple-touch-icon-76x76-precomposed.png">

	<link rel="apple-touch-icon-precomposed" sizes="72x72" href="http://localhost:4000/Race/assets/img/apple-touch-icon-72x72-precomposed.png">

	<link rel="apple-touch-icon-precomposed" href="http://localhost:4000/Race/assets/img/apple-touch-icon-precomposed.png">	

	<meta name="msapplication-TileImage" content="http://localhost:4000/Race/assets/img/msapplication_tileimage.png">

	<meta name="msapplication-TileColor" content="#fabb00">


	

	

		]]></xsl:text>
	</head>
	<body id="top-of-page">
		<xsl:text disable-output-escaping="yes"><![CDATA[
		
<div id="navigation" class="sticky">
  <nav class="top-bar" role="navigation" data-topbar>
    <ul class="title-area">
      <li class="name">
      <h1 class="show-for-small-only"><a href="http://localhost:4000/Race" class="icon-tree"> GRAIC</a></h1>
    </li>
       <!-- Remove the class "menu-icon" to get rid of menu icon. Take out "Menu" to just have icon alone -->
      <li class="toggle-topbar menu-icon"><a href="#"><span>Nav</span></a></li>
    </ul>
    <section class="top-bar-section">

      <ul class="right">
        <li class="divider"></li>
        <li><a href="https://github.com/PoPGRI/Race" target="_blank">Code</a></li>
        <li><a href="mailto:graic2021@gmail.com" target="_blank">Contact</a></li>
        <li><a href="https://groups.google.com/g/graic21" target="_blank">Forum</a></li>

        

              



          
          
        

              



          
          
        

              



          
          
        

              



          
          
        

              



          
          
        

              



          
          
        
        
      </ul>

      <ul class="left">
        

              

          
          

            
            
              <li><a  href="http://localhost:4000/Race/">Home</a></li>
              <li class="divider"></li>

            
            
          
        

              

          
          

            
            

              <li class="has-dropdown">
                <a  href="http://localhost:4000/Race/installation/">Getting Started</a>

                  <ul class="dropdown">
                    

                      

                      <li><a  href="http://localhost:4000/Race/installation/aws/">AWS Installation</a></li>
                    

                      

                      <li><a  href="http://localhost:4000/Race/installation/docker/">Docker Installation</a></li>
                    

                      

                      <li><a  href="http://localhost:4000/Race/installation/running/">Running GRAIC</a></li>
                    
                  </ul>

              </li>
              <li class="divider"></li>
            
          
        

              

          
          

            
            

              <li class="has-dropdown">
                <a  href="http://localhost:4000/Race/documentation/">Docs</a>

                  <ul class="dropdown">
                    

                      

                      <li><a  href="http://localhost:4000/Race/documentation/perception/">Perception</a></li>
                    

                      

                      <li><a  href="http://localhost:4000/Race/documentation/visualization/">Visualization</a></li>
                    

                      

                      <li><a  href="http://localhost:4000/Race/documentation/decision-control/">Decision and Control</a></li>
                    

                      

                      <li><a  href="http://localhost:4000/Race/documentation/scenarios/">Maps and Scenarios</a></li>
                    

                      

                      <li><a  href="http://localhost:4000/Race/documentation/evaluation/">Evaluation</a></li>
                    
                  </ul>

              </li>
              <li class="divider"></li>
            
          
        

              

          
          

            
            
              <li><a  href="http://localhost:4000/Race/submissions/">Submission</a></li>
              <li class="divider"></li>

            
            
          
        

              

          
          

            
            
              <li><a  href="http://localhost:4000/Race/about/">About</a></li>
              <li class="divider"></li>

            
            
          
        

              

          
          

            
            
              <li><a  href="http://localhost:4000/Race/outreach/">Past events</a></li>
              <li class="divider"></li>

            
            
          
        
        
      </ul>
    </section>
  </nav>
</div><!-- /#navigation -->

		







		


<div class="alert-box warning text-center"><p>This <a href="https://en.wikipedia.org/wiki/RSS" target="_blank">Atom feed</a> is meant to be used by <a href="https://en.wikipedia.org/wiki/Template:Aggregators" target="_blank">RSS reader applications and websites</a>.</p>
</div>



		]]></xsl:text>
		<header class="t30 row">
	<p class="subheadline"><xsl:value-of select="atom:subtitle" disable-output-escaping="yes" /></p>
	<h1>
		<xsl:element name="a">
			<xsl:attribute name="href">
				<xsl:value-of select="atom:id" />
			</xsl:attribute>
			<xsl:value-of select="atom:title" />
		</xsl:element>
	</h1>
</header>
<ul class="accordion row" data-accordion="">
	<xsl:for-each select="atom:entry">
		<li class="accordion-navigation">
			<xsl:variable name="slug-id">
				<xsl:call-template name="slugify">
					<xsl:with-param name="text" select="atom:id" />
				</xsl:call-template>
			</xsl:variable>
			<xsl:element name="a">
				<xsl:attribute name="href"><xsl:value-of select="concat('#', $slug-id)"/></xsl:attribute>
				<xsl:value-of select="atom:title"/>
				<br/>
				<small><xsl:value-of select="atom:updated"/></small>
			</xsl:element>
			<xsl:element name="div">
				<xsl:attribute name="id"><xsl:value-of select="$slug-id"/></xsl:attribute>
				<xsl:attribute name="class">content</xsl:attribute>
				<h1>
					<xsl:element name="a">
						<xsl:attribute name="href"><xsl:value-of select="atom:id"/></xsl:attribute>
						<xsl:value-of select="atom:title"/>
					</xsl:element>
				</h1>
				<xsl:value-of select="atom:content" disable-output-escaping="yes" />
			</xsl:element>
		</li>
	</xsl:for-each>
</ul>

		<xsl:text disable-output-escaping="yes"><![CDATA[
		    <div id="up-to-top" class="row">
      <div class="small-12 columns" style="text-align: right;">
        <a class="iconfont" href="#top-of-page">&#xf108;</a>
      </div><!-- /.small-12.columns -->
    </div><!-- /.row -->


    <footer id="footer-content" class="bg-grau">
      <div id="footer">
        <div class="row">
          <div class="medium-6 large-5 columns">
            <h5 class="shadow-black">About This Site</h5>

            <p class="shadow-black">
              Generalized RAcing Intelligence Competition (GRAIC), is a synthesis competition for intelligent racing. 
              <!-- <a href="http://localhost:4000/Race/info/">More ›</a> -->
            </p>
          </div><!-- /.large-6.columns -->


          <div class="small-6 medium-3 large-3 large-offset-1 columns">
            
              
                <h5 class="shadow-black">Site map</h5>
              
            
              
            
              
            
              
            
              
            
              
            

              <ul class="no-bullet shadow-black">
              
                
                  <li >
                    <a href="http://localhost:4000/Race"  title=""></a>
                  </li>
              
                
                  <li >
                    <a href="http://localhost:4000/Race/contact/"  title="Contact">Home</a>
                  </li>
              
                
                  <li >
                    <a href="http://localhost:4000/Race/feed.xml"  title="Subscribe to RSS Feed">Getting Started</a>
                  </li>
              
                
                  <li >
                    <a href="http://localhost:4000/Race/atom.xml"  title="Subscribe to Atom Feed">Docs</a>
                  </li>
              
                
                  <li >
                    <a href="http://localhost:4000/Race/sitemap.xml"  title="Sitemap for Google Webmaster Tools">About</a>
                  </li>
              
                
                  <li >
                    <a href="http://localhost:4000/Race/sitemap.xml"  title="Sitemap for Google Webmaster Tools">Past Events</a>
                  </li>
              
              </ul>
          </div><!-- /.large-4.columns -->


          <div class="small-6 medium-3 large-3 columns">
            
              
                <h5 class="shadow-black">Contact</h5>
              
            
              
            
              
            
              
            

            <ul class="no-bullet shadow-black">
            
              
                <li >
                  <a href="http://localhost:4000/Race"  title=""></a>
                </li>
            
              
                <li >
                  <a href="https://groups.google.com/g/graic21" target="_blank"  title="GRAIC forum">Forum</a>
                </li>
            
              
                <li >
                  <a href="http://localhost:4000/Racemailto:graic2021@gmail.com"  title="Email us!">graic2021@gmail.com</a>
                </li>
            
              
                <li >
                  <a href="https://github.com/PoPGRI/Race" target="_blank"  title="GRAIC source code">GRAIC code</a>
                </li>
            
            </ul>
          </div><!-- /.large-3.columns -->
        </div><!-- /.row -->

      </div><!-- /#footer -->


      <div id="subfooter">
        <nav class="row">
          <section id="subfooter-left" class="small-12 medium-6 columns credits">
            <p>Created by <a href="https://publish.illinois.edu/kristinamiller/">Kristina Miller</a> with <a href="http://jekyllrb.com/" target="_blank">Jekyll</a> based on <a href="http://phlow.github.io/feeling-responsive/">Feeling Responsive</a> by <a href="http://phlow.de/">Phlow</a>.</p>
          </section>

          <section id="subfooter-right" class="small-12 medium-6 columns">
            <ul class="inline-list social-icons">
            
              <li><a href="https://github.com/PoPGRI/Race" target="_blank" class="icon-github" title="GRAIC source code"></a></li>
            
              <li><a href="https://www.youtube.com/channel/UC1wObn3tWYZDkWld-Kg8LRA" target="_blank" class="icon-youtube" title="Videos"></a></li>
            
            </ul>
          </section>
        </nav>
      </div><!-- /#subfooter -->
    </footer>

		


<script src="http://localhost:4000/Race/assets/js/javascript.min.js"></script>












		]]></xsl:text>
	</body>
	</html>
</xsl:template>
<xsl:template name="slugify">
	<xsl:param name="text" select="''" />
	<xsl:variable name="dodgyChars" select="' ,.#_-!?*:;=+|&amp;/\\'" />
	<xsl:variable name="replacementChar" select="'-----------------'" />
	<xsl:variable name="lowercase" select="'abcdefghijklmnopqrstuvwxyz'" />
	<xsl:variable name="uppercase" select="'ABCDEFGHIJKLMNOPQRSTUVWXYZ'" />
	<xsl:variable name="lowercased"><xsl:value-of select="translate( $text, $uppercase, $lowercase )" /></xsl:variable>
	<xsl:variable name="escaped"><xsl:value-of select="translate( $lowercased, $dodgyChars, $replacementChar )" /></xsl:variable>
	<xsl:value-of select="$escaped" />
</xsl:template>
</xsl:stylesheet>
