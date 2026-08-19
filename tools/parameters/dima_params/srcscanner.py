import codecs


class SourceScanner(object):
    """Reads explicit source files and passes their contents to the parser."""

    def ScanFile(self, path, parser):
        """
        Scans provided file and passes its contents to the parser using
        parser.Parse method.
        """

        with codecs.open(path, "r", "utf-8") as source:
            try:
                contents = source.read()
            except Exception:
                contents = ""
                print("Failed reading file: %s, skipping content." % path)
        return parser.Parse(contents)
