---
layout: page
title: "Submission"
header:
   image_fullwidth: "header_unsplash_5.jpg"
permalink: "/submissions/"
---

<div class="wrapper">
  {% for entry in site.submissions %}
  <div>
      {% capture year %}{{entry.date | date: "%Y"}}{% endcapture %}
      {% if year == "2022" %}
          <p>{{entry.content}}</p>
      {% endif %}
  </div>
  {% endfor %}
</div>
