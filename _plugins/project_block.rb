# frozen_string_literal: true

require "yaml"

module Jekyll
  class ProjectBlock < Liquid::Block
    def render(context)
      raw = super
      data = YAML.safe_load(raw) || {}

      payload = {
        "summary" => data["summary"],
        "links" => data["links"] || [],
        "dates" => data["dates"],
        "contributors" => data["contributors"],
        "description" => data["description"] || "",
        "iframe" => data["iframe"],
        "slides" => data["slides"] || []
      }

      site_src = context.registers[:site].source
      partial  = File.read(File.join(site_src, "_includes", "project-details.html"))

      Liquid::Template
        .parse(partial)
        .render!({ "include" => payload }, registers: context.registers)
    end
  end
end

Liquid::Template.register_tag("project", Jekyll::ProjectBlock)
