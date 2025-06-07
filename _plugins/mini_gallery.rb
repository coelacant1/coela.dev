# frozen_string_literal: true

require "yaml"

module Jekyll
  class MiniGalleryBlock < Liquid::Block
    def initialize(tag_name, markup, tokens)
      super
    end

    def render(context)
      raw = super
      slides = YAML.safe_load(raw)

      partial = File.read(
        File.join(context.registers[:site].source, "_includes", "mini-gallery.html")
      )

      payload = { "slides" => slides }
      Liquid::Template.parse(partial).render!(payload, registers: context.registers)
    end
  end
end

Liquid::Template.register_tag("mini_gallery", Jekyll::MiniGalleryBlock)
