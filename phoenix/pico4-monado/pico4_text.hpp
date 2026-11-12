// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Text for the SteamVR panels in any of Steam's languages: Pango lays it out (line breaks, Arabic,
 * Thai, right to left) and fontconfig finds the glyphs. The image's own fonts cover Latin, Greek
 * and Cyrillic; the Steam client brings Noto for every script it shows (its clientui/fonts),
 * which is added to fontconfig's fonts for this process.
 */
#pragma once

#include <cairo/cairo.h>
#include <fontconfig/fontconfig.h>
#include <pango/pangocairo.h>
#include <unistd.h>

#include <string>

namespace pico4_text {

//! The fonts in order: the image's, then the Steam client's for every other script.
constexpr const char *kFamilies = "Adwaita Sans, Go Noto Kurrent-Regular, Go Noto Kurrent, Noto Sans, sans-serif";

/*!
 * Add the Steam client's fonts below @p steam (its install folder) to this process's fonts, once;
 * false while it has none (the client unpacks itself at its first start).
 */
inline bool
add_steam_fonts(const std::string &steam)
{
	static bool added = false;
	if (added) {
		return true;
	}
	const std::string dir = steam + "/clientui/fonts";
	if (access(dir.c_str(), R_OK) != 0) {
		return false;
	}
	FcConfig *config = FcConfigGetCurrent();
	added = FcConfigAppFontAddDir(config, reinterpret_cast<const FcChar8 *>(dir.c_str())) == FcTrue;
	if (added) {
		// Pango's font map caches the fonts it has seen: start again with the new ones.
		pango_cairo_font_map_set_default(nullptr);
	}
	return added;
}

enum Align
{
	kStart,  //!< left, or right in a right-to-left language
	kCenter,
};

/*!
 * @p text at @p size px, wrapped to @p width, its first line's top at @p y; returns the height.
 * @p measure only measures.
 */
inline double
draw(cairo_t *c, const std::string &text, double x, double y, double width, double size, bool bold,
     Align align = kStart, bool measure = false)
{
	PangoLayout *layout = pango_cairo_create_layout(c);
	PangoFontDescription *font = pango_font_description_from_string(kFamilies);
	pango_font_description_set_absolute_size(font, size * PANGO_SCALE);
	pango_font_description_set_weight(font, bold ? PANGO_WEIGHT_BOLD : PANGO_WEIGHT_NORMAL);
	pango_layout_set_font_description(layout, font);
	pango_layout_set_width(layout, (int)(width * PANGO_SCALE));
	pango_layout_set_wrap(layout, PANGO_WRAP_WORD_CHAR);
	pango_layout_set_alignment(layout, align == kCenter ? PANGO_ALIGN_CENTER : PANGO_ALIGN_LEFT);
	pango_layout_set_auto_dir(layout, TRUE);
	pango_layout_set_text(layout, text.c_str(), -1);
	int w, h;
	pango_layout_get_pixel_size(layout, &w, &h);
	if (!measure) {
		cairo_move_to(c, x, y);
		pango_cairo_show_layout(c, layout);
	}
	pango_font_description_free(font);
	g_object_unref(layout);
	return h;
}

/*!
 * One line of @p text, cut with an ellipsis to @p width; with @p natural it starts on the right in a
 * right-to-left language, else always on the left (rows of mixed scripts).
 */
inline void
line(cairo_t *c, const std::string &text, double x, double y, double width, double size, bool bold, bool natural = true)
{
	PangoLayout *layout = pango_cairo_create_layout(c);
	PangoFontDescription *font = pango_font_description_from_string(kFamilies);
	pango_font_description_set_absolute_size(font, size * PANGO_SCALE);
	pango_font_description_set_weight(font, bold ? PANGO_WEIGHT_BOLD : PANGO_WEIGHT_NORMAL);
	pango_layout_set_font_description(layout, font);
	pango_layout_set_width(layout, (int)(width * PANGO_SCALE));
	pango_layout_set_ellipsize(layout, PANGO_ELLIPSIZE_END);
	pango_layout_set_single_paragraph_mode(layout, TRUE);
	pango_layout_set_auto_dir(layout, natural ? TRUE : FALSE);
	pango_layout_set_text(layout, text.c_str(), -1);
	cairo_move_to(c, x, y);
	pango_cairo_show_layout(c, layout);
	pango_font_description_free(font);
	g_object_unref(layout);
}

/*!
 * An input field's one line of @p text from the left, its end in view: what is wider than @p width
 * leaves on the left, as a field scrolls while one types. Returns the width drawn, for the caret.
 */
inline double
field(cairo_t *c, const std::string &text, double x, double y, double width, double size)
{
	PangoLayout *layout = pango_cairo_create_layout(c);
	PangoFontDescription *font = pango_font_description_from_string(kFamilies);
	pango_font_description_set_absolute_size(font, size * PANGO_SCALE);
	pango_layout_set_font_description(layout, font);
	pango_layout_set_single_paragraph_mode(layout, TRUE);
	pango_layout_set_auto_dir(layout, FALSE);
	pango_layout_set_text(layout, text.c_str(), -1);
	int w = 0, h = 0;
	pango_layout_get_pixel_size(layout, &w, &h);
	const double shift = w > width ? w - width : 0;
	cairo_save(c);
	cairo_rectangle(c, x, y - size, width, h + 2 * size);
	cairo_clip(c);
	cairo_move_to(c, x - shift, y);
	pango_cairo_show_layout(c, layout);
	cairo_restore(c);
	pango_font_description_free(font);
	g_object_unref(layout);
	return w > width ? width : w;
}

} // namespace pico4_text
