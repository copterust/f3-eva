;;; Directory Local Variables
;;; For more information see (info "(emacs) Directory Variables")

((rust-mode
  (flycheck-rust-check-tests)
  (flycheck-checker . rust-xargo)
  (lsp-rust-sysroot . "/Users/r/.xargo")
  (lsp-rust-taget . "thumbv7m-none-eabi")
  (flycheck-rust-crate-type . "bin")
  (flycheck-rust-binary-name . "f3_eva")))
