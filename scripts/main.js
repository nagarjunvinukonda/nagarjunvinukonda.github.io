
// Preserve the exact position on a browser refresh.
(function () {
  var key = 'portfolio-scroll-y';
  var navEntries = window.performance && performance.getEntriesByType
    ? performance.getEntriesByType('navigation')
    : [];
  var isReload = navEntries.length
    ? navEntries[0].type === 'reload'
    : (window.performance && performance.navigation && performance.navigation.type === 1);

  if (isReload && 'scrollRestoration' in history) {
    history.scrollRestoration = 'manual';
  }

  var ticking = false;
  window.addEventListener('scroll', function () {
    if (!ticking) {
      window.requestAnimationFrame(function () {
        sessionStorage.setItem(key, String(window.pageYOffset || document.documentElement.scrollTop || 0));
        ticking = false;
      });
      ticking = true;
    }
  }, { passive: true });

  if (isReload) {
    var saved = parseFloat(sessionStorage.getItem(key));
    if (!isNaN(saved)) {
      window.addEventListener('load', function () {
        window.requestAnimationFrame(function () { window.scrollTo(0, saved); });
        window.setTimeout(function () { window.scrollTo(0, saved); }, 250);
      });
    }
  }
})();
// End refresh-position preservation.

// Add your javascript here
// Don't forget to add it into respective layouts where this js file is needed

$(document).ready(function() {
  AOS.init( {
    // uncomment below for on-scroll animations to played only once
    // once: true  
  }); // initialize animate on scroll library
});

// Smooth scroll for links with hashes
$('a.smooth-scroll')
.click(function(event) {
  // On-page links
  if (
    location.pathname.replace(/^\//, '') == this.pathname.replace(/^\//, '') 
    && 
    location.hostname == this.hostname
  ) {
    // Figure out element to scroll to
    var target = $(this.hash);
    target = target.length ? target : $('[name=' + this.hash.slice(1) + ']');
    // Does a scroll target exist?
    if (target.length) {
      // Only prevent default if animation is actually gonna happen
      event.preventDefault();
      $('html, body').animate({
        scrollTop: target.offset().top
      }, 1000, function() {
        // Callback after animation
        // Must change focus!
        var $target = $(target);
        $target.focus();
        if ($target.is(":focus")) { // Checking if the target was focused
          return false;
        } else {
          $target.attr('tabindex','-1'); // Adding tabindex for elements not focusable
          $target.focus(); // Set focus again
        };
      });
    }
  }
});
