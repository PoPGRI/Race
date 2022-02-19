---
layout: page
title: "About"
permalink: "/about/"
---

We would like to thank all the organizors and participants that make GRAIC possible!

<div class="wrapper">
  {% for entry in site.about %}
  <div>
      {% capture year %}{{entry.date | date: "%Y"}}{% endcapture %}
      {% if year == "2022" %}
          {% if entry.title != blank %}
          <h1>{{entry.title}}</h1>
          {% endif %}
          <p>{{entry.content}}</p>
      {% endif %}
  </div>
  {% endfor %}
</div>
